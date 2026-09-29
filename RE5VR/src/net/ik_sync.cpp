// Winsock FIRST, and that is not a style choice. windows.h pulls in the
// original winsock.h, which collides with winsock2.h line for line, and every
// project header below reaches windows.h eventually. Including these two up
// here is what stops a hundred errors inside ws2tcpip.h.
#include <winsock2.h>
#include <ws2tcpip.h>

#include "ik_sync.h"

#include "../hooks/arm_ik.h"
#include "../util/log.h"
#include "../vr/openxr_bridge.h"
#include "../proxy/real_d3d9.h"
#include "../vr/xr_input.h"

#include <MinHook.h>

#include <cmath>
#include <cstring>
#include <cstdio>

#pragma comment(lib, "ws2_32.lib")

// See ik_sync.h for what this carries and why it carries those numbers.
//
// Everything that can block lives on the worker thread. The game thread only
// ever takes a lock long enough to copy a hundred-odd bytes, which is the one
// rule that matters: a stalled network must never become a stalled frame.

namespace {

constexpr unsigned kMagic = 0x4B493552; // "R5IK"
// 2: the packet gained the sender's units-per-metre, without which the
// receiver cannot turn their reach in metres into their character's units.
constexpr unsigned short kVersion = 2;
// Ninety a second, not thirty (2026-09-24, user: "why are we capping to 30
// packets? Why not up it and give more data?"). There was never a reason for
// thirty beyond caution, and the arithmetic does not support the caution: the
// packet is about 140 bytes, so thirty a second is 4 KB/s and ninety is 13,
// against a game already streaming its own state between the two machines.
//
// Being stingy cost real fidelity. A quarter of these go missing - the log
// reads "sent 27, 20 arrived" - so the receiver was getting fresh hands about
// twenty times a second while drawing a hundred and twenty frames, which is
// five frames in six showing a pose that is already old. Sending three times
// as often also means a lost packet costs a third as much, because the next
// one is a third as far away.
//
// This is not a substitute for smoothing between packets, which is still
// worth doing. It just moves the floor a long way up first.
constexpr int kSendHz = 90;
constexpr unsigned long long kStaleMs = 500;

#pragma pack(push, 1)
struct Packet {
    unsigned magic;
    unsigned short version;
    // bit 0/1 hand left/right, bit 2/3 shoulder left/right, bit 4 both hands
    // on the gun (2026-09-23). Their support hand is snapped onto their gun
    // AFTER this packet is built, by a solve that needs their skeleton and
    // their weapon, so the corrected wrist never travels and their off hand
    // arrives holding nothing. Sending the fact instead of the result lets
    // the receiver run the same snap against the gun IT can see, which is the
    // one actually in the partner's hands on this machine.
    unsigned short flags;
    unsigned seq;
    float handFromHead[2][3];
    float shoulderFromHead[2][3];
    float basis[2][9];
    float unitsPerMetre[2];
};
#pragma pack(pop)

SRWLOCK g_lock = SRWLOCK_INIT;
IkSyncSettings g_settings;
bool g_wantRunning = false;

HANDLE g_thread = nullptr;
volatile LONG g_stop = 0;

IkSyncHands g_partner = {};
// When each hand was last actually described, so one can be held while the
// other keeps updating.
unsigned long long g_partnerHandMs[2] = {};
bool g_havePartner = false;
char g_status[160] = "off";

// Where to send. Either the address the settings name, or wherever the last
// packet came from - so only one of the two players has to know the other's
// address, and the one behind the harder NAT can be the one who types it.
sockaddr_in g_peer = {};
bool g_havePeer = false;
bool g_peerLearned = false;

// ---- Finding the partner without being told (2026-09-19) -------------------
// Typing in an address is a poor way to start a game, and the user said so:
// "it seems odd to have to type an address in. Is there no way we can just pick
// up on your multiplayer partner?"
//
// There is, and it does not need the game's netcode understood. Whoever RE5 is
// already talking to IS the partner, and everything it says goes out through
// ws2_32. So watch where it sends. This does not touch, alter, delay or read
// one byte of the game's traffic: it takes the destination address off the call
// on its way past and hands the call straight on.
//
// The catch, and it is a real one, is that Steam may be relaying rather than
// connecting the two of you directly. Then the busiest address is a Valve
// machine, our packets to it go nowhere, and the typed address is still the way
// in. Which of those it is cannot be reasoned out from here, so the candidates
// go in the log and one co-op session says.
//
// Both ends run this mod - they have to, since each solves its own arms - so
// both will find each other and both will start sending at once. Two ends
// sending at once is how a UDP hole gets punched, which the typed address
// alone would not manage either.

struct Candidate {
    unsigned long addr; // network order
    volatile LONG hits;
};
constexpr int kCandidates = 8;
Candidate g_seen[kCandidates] = {};
volatile LONG g_watching = 0;
// Counters, because "found nobody" has three completely different causes and
// the first session could not tell them apart (2026-09-19). Either the game
// never sends through us at all, or it does and everything is being filtered,
// or it does and no one destination is busy enough to commit to. Those want
// three different fixes, so they get counted separately.
volatile LONG g_callsSeen = 0;
volatile LONG g_dropLocal = 0;
volatile LONG g_dropOurs = 0;
volatile LONG g_dropFamily = 0;
volatile LONG g_dropFull = 0;
unsigned short g_ourPortNet = 0;
bool g_hooksInstalled = false;
// True only when the DLL's own strings name the interface version this code
// was written against. Nothing gets patched otherwise: a wrong vtable index
// does not fail politely.
bool g_netVersionOk = false;

typedef int(WSAAPI* sendto_t)(SOCKET, const char*, int, int, const sockaddr*, int);
typedef int(WSAAPI* wsasendto_t)(SOCKET, LPWSABUF, DWORD, LPDWORD, DWORD, const sockaddr*, int,
    LPWSAOVERLAPPED, LPWSAOVERLAPPED_COMPLETION_ROUTINE);
typedef int(WSAAPI* connect_t)(SOCKET, const sockaddr*, int);
sendto_t g_origSendTo = nullptr;
wsasendto_t g_origWsaSendTo = nullptr;
connect_t g_origConnect = nullptr;
typedef int(WSAAPI* send_t)(SOCKET, const char*, int, int);
typedef int(WSAAPI* wsasend_t)(SOCKET, LPWSABUF, DWORD, LPDWORD, DWORD, LPWSAOVERLAPPED,
    LPWSAOVERLAPPED_COMPLETION_ROUTINE);
send_t g_origSend = nullptr;
wsasend_t g_origWsaSend = nullptr;

// Deliberately tiny, and it never logs, allocates or takes a lock: it runs on
// the game's own network path.
void NoteDestination(const sockaddr* to, int len)
{
    if (InterlockedCompareExchange(&g_watching, 0, 0) == 0)
        return;
    InterlockedIncrement(&g_callsSeen);
    if (!to || len < static_cast<int>(sizeof(sockaddr_in)) || to->sa_family != AF_INET) {
        InterlockedIncrement(&g_dropFamily);
        return;
    }
    const sockaddr_in* in4 = reinterpret_cast<const sockaddr_in*>(to);
    // Our own sync traffic must not end up in its own list of candidates.
    if (in4->sin_port == g_ourPortNet) {
        InterlockedIncrement(&g_dropOurs);
        return;
    }
    const unsigned long host = ntohl(in4->sin_addr.s_addr);
    const unsigned char top = static_cast<unsigned char>(host >> 24);
    if (top == 127 || top == 0 || top >= 224) { // loopback, unspecified, multicast and broadcast
        InterlockedIncrement(&g_dropLocal);
        return;
    }

    const unsigned long addr = in4->sin_addr.s_addr;
    for (int i = 0; i < kCandidates; ++i) {
        if (g_seen[i].addr == addr) {
            InterlockedIncrement(&g_seen[i].hits);
            return;
        }
    }
    for (int i = 0; i < kCandidates; ++i) {
        if (g_seen[i].addr == 0) {
            g_seen[i].addr = addr;
            InterlockedIncrement(&g_seen[i].hits);
            return;
        }
    }
    InterlockedIncrement(&g_dropFull);
}

// Everything the watcher has seen, in one line. Printed while it is still
// looking, because a search that finds nothing has to say what it looked at.
void ReportWatch()
{
    char list[320] = "";
    size_t used = 0;
    for (int i = 0; i < kCandidates; ++i) {
        if (!g_seen[i].addr)
            continue;
        char name[64] = "";
        inet_ntop(AF_INET, &g_seen[i].addr, name, sizeof(name));
        const int wrote = _snprintf_s(list + used, sizeof(list) - used, _TRUNCATE, "%s%s (%ld)",
            used ? ", " : "", name, g_seen[i].hits);
        if (wrote <= 0)
            break;
        used += static_cast<size_t>(wrote);
    }
    Log_Printf("IkSync: the game made %ld sends - %ld local, %ld not IPv4, %ld ours, %ld past the list. "
               "Candidates: %s",
        g_callsSeen, g_dropLocal, g_dropFamily, g_dropOurs, g_dropFull, used ? list : "none");
}

int WSAAPI HookSendTo(SOCKET s, const char* buf, int len, int flags, const sockaddr* to, int tolen)
{
    NoteDestination(to, tolen);
    return g_origSendTo(s, buf, len, flags, to, tolen);
}

int WSAAPI HookWsaSendTo(SOCKET s, LPWSABUF bufs, DWORD count, LPDWORD sent, DWORD flags, const sockaddr* to,
    int tolen, LPWSAOVERLAPPED ov, LPWSAOVERLAPPED_COMPLETION_ROUTINE routine)
{
    NoteDestination(to, tolen);
    return g_origWsaSendTo(s, bufs, count, sent, flags, to, tolen, ov, routine);
}

int WSAAPI HookConnect(SOCKET s, const sockaddr* to, int tolen)
{
    NoteDestination(to, tolen);
    return g_origConnect(s, to, tolen);
}

// A socket that has been connected does not carry a destination on every send,
// so watching sendto alone misses it entirely (2026-09-19). That is a real
// possibility for how RE5 talks and would look exactly like the game never
// sending at all. The socket knows its own far end, so ask it.
void NoteConnected(SOCKET s)
{
    if (InterlockedCompareExchange(&g_watching, 0, 0) == 0)
        return;
    sockaddr_in peer = {};
    int len = sizeof(peer);
    if (getpeername(s, reinterpret_cast<sockaddr*>(&peer), &len) == 0)
        NoteDestination(reinterpret_cast<const sockaddr*>(&peer), len);
}

int WSAAPI HookSend(SOCKET s, const char* buf, int len, int flags)
{
    NoteConnected(s);
    return g_origSend(s, buf, len, flags);
}

int WSAAPI HookWsaSend(SOCKET s, LPWSABUF bufs, DWORD count, LPDWORD sent, DWORD flags,
    LPWSAOVERLAPPED ov, LPWSAOVERLAPPED_COMPLETION_ROUTINE routine)
{
    NoteConnected(s);
    return g_origWsaSend(s, bufs, count, sent, flags, ov, routine);
}

void InstallWatch()
{
    // Installed once and left in place. Taking a hook back out while the game
    // is mid-send is a worse risk than leaving a branch that does nothing, so
    // switching this off only clears g_watching.
    if (g_hooksInstalled)
        return;
    g_hooksInstalled = true;

    HMODULE ws2 = GetModuleHandleA("ws2_32.dll");
    if (!ws2)
        ws2 = LoadLibraryA("ws2_32.dll");
    if (!ws2) {
        Log_Printf("IkSync: no ws2_32, so the partner has to be typed in");
        return;
    }

    struct Target {
        const char* name;
        void* detour;
        void** original;
    };
    const Target targets[] = {
        { "sendto", reinterpret_cast<void*>(&HookSendTo), reinterpret_cast<void**>(&g_origSendTo) },
        { "WSASendTo", reinterpret_cast<void*>(&HookWsaSendTo), reinterpret_cast<void**>(&g_origWsaSendTo) },
        { "connect", reinterpret_cast<void*>(&HookConnect), reinterpret_cast<void**>(&g_origConnect) },
        { "send", reinterpret_cast<void*>(&HookSend), reinterpret_cast<void**>(&g_origSend) },
        { "WSASend", reinterpret_cast<void*>(&HookWsaSend), reinterpret_cast<void**>(&g_origWsaSend) },
    };
    int installed = 0;
    for (const Target& t : targets) {
        void* fn = reinterpret_cast<void*>(GetProcAddress(ws2, t.name));
        if (!fn)
            continue;
        const MH_STATUS st = MH_CreateHook(fn, t.detour, t.original);
        if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED)
            continue;
        if (MH_EnableHook(fn) == MH_OK)
            ++installed;
    }
    Log_Printf("IkSync: watching %d of 5 send paths to find who the game is talking to", installed);
}

// The busiest address the game has been talking to. Ties and near-ties are
// reported rather than resolved, because a relay and a partner look identical
// from here and only a real session can tell them apart.
bool BestCandidate(unsigned long* outAddr, char* why, size_t whySize)
{
    int best = -1, second = -1;
    for (int i = 0; i < kCandidates; ++i) {
        if (!g_seen[i].addr)
            continue;
        if (best < 0 || g_seen[i].hits > g_seen[best].hits) {
            second = best;
            best = i;
        } else if (second < 0 || g_seen[i].hits > g_seen[second].hits) {
            second = i;
        }
    }
    if (best < 0 || g_seen[best].hits < 20) // a handful of packets is not a session
        return false;

    char bestName[64] = "";
    inet_ntop(AF_INET, &g_seen[best].addr, bestName, sizeof(bestName));
    if (second >= 0) {
        char secondName[64] = "";
        inet_ntop(AF_INET, &g_seen[second].addr, secondName, sizeof(secondName));
        _snprintf_s(why, whySize, _TRUNCATE, "%s (%ld packets); next busiest %s (%ld)", bestName,
            g_seen[best].hits, secondName, g_seen[second].hits);
    } else {
        _snprintf_s(why, whySize, _TRUNCATE, "%s (%ld packets)", bestName, g_seen[best].hits);
    }
    *outAddr = g_seen[best].addr;
    return true;
}

// Milliseconds that actually move in milliseconds.
double NowMs()
{
    static LARGE_INTEGER freq = {};
    if (!freq.QuadPart)
        QueryPerformanceFrequency(&freq);
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
}

// ---- Is Steam's own P2P reachable from here? (2026-09-19) ------------------
// Two co-op sessions proved that a plain UDP socket cannot find the partner.
// The game's traffic never passes through this process, and when both ends
// were told each other's address by hand, both sent and neither received -
// which is what a NAT that does not preserve port numbers looks like, and
// nothing either player types can fix it.
//
// But steam_api.dll, steamclient.dll, tier0_s.dll and vstdlib_s.dll are all
// loaded in this process. If that steam_api belongs to the GAME, then Steam's
// own peer-to-peer channel is reachable from in here, and it makes the whole
// problem disappear: Valve already solved NAT traversal, relaying and
// addressing, and a packet is sent to a SteamID rather than to an address.
//
// The catch is that steam_api.dll is NOT in the RE5 folder, and SteamVR ships
// one too. If what is loaded came in with the headset rather than the game,
// it is the wrong Steam session and none of this works.
//
// So: read, and do not touch. Nothing here calls SteamAPI_Init, because the
// game owns that and initialising Steam underneath it is a good way to break
// something that currently works. Asking for the user handle is enough - it is
// non-zero only if somebody in this process has already brought Steam up.
void ProbeSteam()
{
    HMODULE api = GetModuleHandleA("steam_api.dll");
    if (!api) {
        Log_Printf("SteamProbe: steam_api.dll is not loaded, so there is no Steam route from here");
        return;
    }
    char path[MAX_PATH] = "";
    GetModuleFileNameA(api, path, sizeof(path));
    Log_Printf("SteamProbe: steam_api.dll loaded from %s", path);

    // Every name it exports, read out of the loaded image.
    //
    // Not off the file, because there is no file: steam_api.dll is unpacked
    // when the game starts and taken away again when it exits, so the only
    // copy that exists is the one in memory (2026-09-19). And not by guessing
    // names either - the first probe asked for nine and found three, which
    // told us the SDK is old without telling us what it DOES have. An old SDK
    // exports its interfaces under different names, so the list is the answer.
    {
        const BYTE* base = reinterpret_cast<const BYTE*>(api);
        const IMAGE_DOS_HEADER* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
        const IMAGE_NT_HEADERS* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
        const DWORD rva = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT].VirtualAddress;
        if (rva) {
            const IMAGE_EXPORT_DIRECTORY* dir
                = reinterpret_cast<const IMAGE_EXPORT_DIRECTORY*>(base + rva);
            const DWORD* names = reinterpret_cast<const DWORD*>(base + dir->AddressOfNames);
            char line[320] = "";
            size_t used = 0;
            for (DWORD i = 0; i < dir->NumberOfNames; ++i) {
                const char* name = reinterpret_cast<const char*>(base + names[i]);
                const size_t len = strlen(name);
                if (used + len + 2 >= sizeof(line)) {
                    Log_Printf("SteamProbe: exports %s", line);
                    used = 0;
                    line[0] = '\0';
                }
                if (used) {
                    line[used++] = ' ';
                    line[used] = '\0';
                }
                memcpy(line + used, name, len + 1);
                used += len;
            }
            if (used)
                Log_Printf("SteamProbe: exports %s", line);
            Log_Printf("SteamProbe: %lu exports in total", dir->NumberOfNames);
        }
    }

    static const char* const kWanted[] = {
        "SteamAPI_Init",
        "SteamAPI_GetHSteamUser",
        "SteamAPI_GetHSteamPipe",
        "SteamInternal_CreateInterface",
        "SteamAPI_ISteamNetworking_SendP2PPacket",
        "SteamAPI_ISteamNetworkingMessages_SendMessageToUser",
        "SteamAPI_ISteamUser_GetSteamID",
        "SteamAPI_ISteamFriends_GetCoplayFriendCount",
        "SteamAPI_ISteamMatchmaking_GetLobbyMemberByIndex",
    };
    char have[512] = "";
    char missing[512] = "";
    size_t haveUsed = 0, missUsed = 0;
    for (const char* name : kWanted) {
        const bool found = GetProcAddress(api, name) != nullptr;
        char* buf = found ? have : missing;
        size_t& used = found ? haveUsed : missUsed;
        const int wrote = _snprintf_s(buf + used, 512 - used, _TRUNCATE, "%s%s", used ? " " : "", name);
        if (wrote > 0)
            used += static_cast<size_t>(wrote);
    }
    Log_Printf("SteamProbe: has %s", haveUsed ? have : "none of what was asked for");
    if (missUsed)
        Log_Printf("SteamProbe: missing %s", missing);

    // Which VERSION of each interface this build speaks (2026-09-19).
    //
    // The accessors are exported, so SteamNetworking() hands back a pointer -
    // but a pointer to what? Every Steam interface is versioned and the vtable
    // order changes between versions, so calling SendP2PPacket at the wrong
    // slot does not fail politely, it calls whatever is there. Guessing is not
    // an option.
    //
    // No need to guess: Steamworks keeps the version strings in the DLL, one
    // per interface, so reading them off is exact. Scanning the image for them
    // touches nothing and cannot crash.
    {
        const BYTE* base = reinterpret_cast<const BYTE*>(api);
        const IMAGE_DOS_HEADER* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
        const IMAGE_NT_HEADERS* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
        const size_t span = nt->OptionalHeader.SizeOfImage;
        char line[320] = "";
        size_t used = 0;
        int found = 0;
        for (size_t at = 0; at + 12 < span && found < 40; ++at) {
            if (memcmp(base + at, "Steam", 5) != 0)
                continue;
            // A version string is letters then exactly three digits, and it is
            // short. Anything else that starts with "Steam" is not one.
            size_t len = 0;
            while (len < 40 && base[at + len] >= 32 && base[at + len] < 127)
                ++len;
            if (len < 12 || len > 32 || base[at + len] != 0)
                continue;
            const char* s = reinterpret_cast<const char*>(base + at);
            if (strcmp(s, "SteamNetworking005") == 0)
                g_netVersionOk = true;
            if (!(s[len - 1] >= '0' && s[len - 1] <= '9') || !(s[len - 2] >= '0' && s[len - 2] <= '9')
                || !(s[len - 3] >= '0' && s[len - 3] <= '9'))
                continue;
            if (used + len + 2 >= sizeof(line)) {
                Log_Printf("SteamProbe: interfaces %s", line);
                used = 0;
                line[0] = '\0';
            }
            if (used) {
                line[used++] = ' ';
                line[used] = '\0';
            }
            memcpy(line + used, s, len + 1);
            used += len;
            ++found;
            at += len;
        }
        if (used)
            Log_Printf("SteamProbe: interfaces %s", line);
    }

    // And whether the accessors actually hand anything back. Calling these is
    // safe: they take no arguments and return a pointer, with no vtable call
    // involved, so a null here costs nothing and tells us plenty.
    {
        typedef void*(__cdecl * Accessor_t)();
        static const char* const kAccessors[]
            = { "SteamClient", "SteamUser", "SteamFriends", "SteamUtils", "SteamMatchmaking", "SteamNetworking" };
        char line[320] = "";
        size_t used = 0;
        for (const char* name : kAccessors) {
            const auto fn = reinterpret_cast<Accessor_t>(GetProcAddress(api, name));
            void* got = fn ? fn() : nullptr;
            const int wrote = _snprintf_s(line + used, sizeof(line) - used, _TRUNCATE, "%s%s=%p",
                used ? " " : "", name, got);
            if (wrote > 0)
                used += static_cast<size_t>(wrote);
        }
        Log_Printf("SteamProbe: %s", line);
    }

    // The one that matters. Non-zero means Steam is already up in this process
    // and belongs to somebody here - almost certainly the game.
    typedef int(__cdecl * GetHSteamUser_t)();
    const auto getUser = reinterpret_cast<GetHSteamUser_t>(GetProcAddress(api, "SteamAPI_GetHSteamUser"));
    typedef int(__cdecl * GetHSteamPipe_t)();
    const auto getPipe = reinterpret_cast<GetHSteamPipe_t>(GetProcAddress(api, "SteamAPI_GetHSteamPipe"));
    if (getUser && getPipe) {
        const int user = getUser();
        const int pipe = getPipe();
        Log_Printf("SteamProbe: user handle %d, pipe handle %d - %s", user, pipe,
            (user && pipe) ? "Steam is live in this process, so the P2P route is open"
                           : "nobody has started Steam in this process, so this steam_api is a passenger");
    }
}

// ---- Who is the partner, asked at the right layer (2026-09-19) -------------
// The winsock watcher found nothing because RE5 does not send through winsock:
// it sends through Steam. So ask Steam instead. ISteamNetworking005's very
// first vtable entry is
//
//   SendP2PPacket(CSteamID remote, const void*, uint32, EP2PSend, int channel)
//
// and the game calls it with the partner's SteamID every time it says anything
// to them. Watching that one call is the whole answer: no address, no port, no
// friends list, no lobby, no guessing.
//
// Version-pinned rather than assumed. The strings in the DLL say
// SteamNetworking005, which is the version whose vtable begins with
// SendP2PPacket. On any other version this is left alone, because a wrong
// vtable index does not fail politely.
//
// __thiscall on x86 puts "this" in ecx and everything else on the stack, which
// is what __fastcall with an ignored edx gives us. The call is passed straight
// through untouched: nothing is added, dropped, delayed or rewritten.
typedef bool(__fastcall* SendP2P_t)(void* self, void* edx, unsigned long long steamID, const void* data,
    unsigned int size, int sendType, int channel);
SendP2P_t g_origSendP2P = nullptr;
volatile LONG g_partnerLow = 0, g_partnerHigh = 0;
volatile LONG g_p2pCalls = 0;

bool __fastcall HookSendP2P(void* self, void* edx, unsigned long long steamID, const void* data,
    unsigned int size, int sendType, int channel)
{
    InterlockedIncrement(&g_p2pCalls);
    // Two halves, stored separately, because a 64-bit write is not atomic on
    // x86 and a torn SteamID is an address for somebody who does not exist.
    // Only the reader has to care, and it re-reads until the halves agree.
    InterlockedExchange(&g_partnerHigh, static_cast<LONG>(steamID >> 32));
    InterlockedExchange(&g_partnerLow, static_cast<LONG>(steamID & 0xFFFFFFFFu));
    return g_origSendP2P(self, edx, steamID, data, size, sendType, channel);
}

// The interface itself, plus the three vtable entries the transport needs.
// Slots 1 and 2 of SteamNetworking005 are IsP2PPacketAvailable and
// ReadP2PPacket, and slot 3 is AcceptP2PSessionWithUser. Taken once, when the
// version has already been checked, so nothing here is ever called on an
// interface it was not written for.
void* g_steamNet = nullptr;
typedef bool(__fastcall* IsP2PAvail_t)(void* self, void* edx, unsigned* size, int channel);
typedef bool(__fastcall* ReadP2P_t)(void* self, void* edx, void* dest, unsigned destSize, unsigned* msgSize,
    unsigned long long* remote, int channel);
typedef bool(__fastcall* AcceptP2P_t)(void* self, void* edx, unsigned long long steamID);
IsP2PAvail_t g_isP2PAvail = nullptr;
ReadP2P_t g_readP2P = nullptr;
AcceptP2P_t g_acceptP2P = nullptr;

// A channel of our own. RE5's own traffic is on whatever channel it chose, and
// ReadP2PPacket only ever hands back packets from the channel asked for, so
// the two streams cannot see each other even though they share a session.
constexpr int kArmChannel = 47;
unsigned g_steamSent = 0, g_steamRaw = 0, g_steamGot = 0;
unsigned g_steamBadSize = 0, g_steamNotOurs = 0, g_steamBadVersion = 0, g_steamInsane = 0;
// The last full second's worth, kept so the menu can show a steady number
// instead of a counter being reset under it.
volatile LONG g_steamSentLast = 0, g_steamGotLast = 0;

bool WatchSteamPartner()
{
    HMODULE api = GetModuleHandleA("steam_api.dll");
    if (!api)
        return false;
    typedef void*(__cdecl * Accessor_t)();
    const auto getNet = reinterpret_cast<Accessor_t>(GetProcAddress(api, "SteamNetworking"));
    void* net = getNet ? getNet() : nullptr;
    if (!net) {
        Log_Printf("SteamP2P: no ISteamNetworking, so the partner cannot be found this way");
        return false;
    }
    if (g_origSendP2P)
        return true;
    if (!g_netVersionOk) {
        Log_Printf("SteamP2P: this is not SteamNetworking005, so the vtable is left alone");
        return false;
    }

    void** vtable = *reinterpret_cast<void***>(net);
    DWORD old = 0;
    if (!VirtualProtect(&vtable[0], sizeof(void*), PAGE_READWRITE, &old)) {
        Log_Printf("SteamP2P: could not make the interface writable (%lu)", GetLastError());
        return false;
    }
    g_origSendP2P = reinterpret_cast<SendP2P_t>(vtable[0]);
    vtable[0] = reinterpret_cast<void*>(&HookSendP2P);
    VirtualProtect(&vtable[0], sizeof(void*), old, &old);

    // The rest of the transport, taken while the version is known good.
    g_steamNet = net;
    g_isP2PAvail = reinterpret_cast<IsP2PAvail_t>(vtable[1]);
    g_readP2P = reinterpret_cast<ReadP2P_t>(vtable[2]);
    g_acceptP2P = reinterpret_cast<AcceptP2P_t>(vtable[3]);
    Log_Printf("SteamP2P: watching SendP2PPacket at %p - the partner will name themselves the moment the "
               "game speaks to them",
        g_origSendP2P);
    return true;
}

// The partner's SteamID, or 0 before the game has spoken to anybody.
// Whoever has actually sent us arms (2026-09-23). The partner is worked out by
// watching who the GAME sends packets to, and a real session showed the game
// talking to three different SteamIDs - 862 sends to one, 815 to another, 211
// to a third - in a two player game. Lobbies, voice and Valve's own services
// all live on that call, so the last one seen is a coin toss, and two thirds
// of our arms went to somebody who was not running the mod.
//
// One packet that passes the magic, the version and the sanity check settles
// it for good: only a machine running this mod can produce one. From then on
// the watcher's opinion is not needed.
volatile LONG g_pinnedHigh = 0, g_pinnedLow = 0;

void PinPartner(unsigned long long who)
{
    if (!who)
        return;
    const LONG high = static_cast<LONG>(static_cast<unsigned long>(who >> 32));
    const LONG low = static_cast<LONG>(static_cast<unsigned long>(who & 0xFFFFFFFFull));
    if (InterlockedCompareExchange(&g_pinnedHigh, 0, 0) == high
        && InterlockedCompareExchange(&g_pinnedLow, 0, 0) == low)
        return;
    InterlockedExchange(&g_pinnedHigh, high);
    InterlockedExchange(&g_pinnedLow, low);
    Log_Printf("SteamP2P: %llu is the partner - they sent arms, so they are running the mod. Everyone else the "
               "game talks to is ignored from here.",
        who);
}

unsigned long long PartnerSteamID()
{
    {
        const LONG high = InterlockedCompareExchange(&g_pinnedHigh, 0, 0);
        const LONG low = InterlockedCompareExchange(&g_pinnedLow, 0, 0);
        if (high || low)
            return (static_cast<unsigned long long>(static_cast<unsigned long>(high)) << 32)
                | static_cast<unsigned long>(low);
    }
    for (int tries = 0; tries < 4; ++tries) {
        const LONG high = InterlockedCompareExchange(&g_partnerHigh, 0, 0);
        const LONG low = InterlockedCompareExchange(&g_partnerLow, 0, 0);
        const LONG check = InterlockedCompareExchange(&g_partnerHigh, 0, 0);
        if (high == check)
            return (static_cast<unsigned long long>(static_cast<unsigned long>(high)) << 32)
                | static_cast<unsigned long>(low);
    }
    return 0;
}

void SetStatus(const char* text)
{
    AcquireSRWLockExclusive(&g_lock);
    strcpy_s(g_status, text);
    ReleaseSRWLockExclusive(&g_lock);
}

// The local hands, in the same terms the packet carries. Built here rather than
// inside the solver so that switching this on cannot disturb the solve: it
// reads the same sources the solver reads and writes nothing back.
// Fills in whatever this end actually has. A player who is not in VR has no
// hands to describe, and that is a perfectly good thing to send: the packet
// still carries the address and still holds the NAT open. Returns whether any
// hand went in, for the status line only.
bool BuildLocal(Packet& p)
{
    float frame[9];
    XRBridgeEyeView eyeL, eyeR;
    if (!VRBridge_GetTrackingFrame(frame) || !VRBridge_GetEyeViews(eyeL, eyeR))
        return false;

    const float head[3] = { (eyeL.positionMeters[0] + eyeR.positionMeters[0]) * 0.5f,
        (eyeL.positionMeters[1] + eyeR.positionMeters[1]) * 0.5f,
        (eyeL.positionMeters[2] + eyeR.positionMeters[2]) * 0.5f };

    bool any = false;
    for (int hand = 0; hand < 2; ++hand) {
        XrHandPose pose;
        if (!XrInput_GetHandPose(hand, pose) || !pose.tracked)
            continue;
        const float raw[3] = { pose.posMeters[0] - head[0], pose.posMeters[1] - head[1],
            pose.posMeters[2] - head[2] };
        // Into the recentred frame: how far right, up and in front of their own
        // head the hand is being held.
        for (int axis = 0; axis < 3; ++axis) {
            p.handFromHead[hand][axis] = raw[0] * frame[axis * 3] + raw[1] * frame[axis * 3 + 1]
                + raw[2] * frame[axis * 3 + 2];
        }
        // The controller's own axes, carried into that frame the same way.
        for (int r = 0; r < 3; ++r) {
            const float* row = pose.rot + r * 3;
            for (int axis = 0; axis < 3; ++axis) {
                p.basis[hand][r * 3 + axis] = row[0] * frame[axis * 3] + row[1] * frame[axis * 3 + 1]
                    + row[2] * frame[axis * 3 + 2];
            }
        }
        p.flags |= static_cast<unsigned short>(1u << hand);
        any = true;
        if (XrInput_GetTwoHand(nullptr))
            p.flags |= 16u;

        float shoulder[3];
        if (ArmIk_GetShoulderFromHead(hand, shoulder)) {
            std::memcpy(p.shoulderFromHead[hand], shoulder, sizeof(shoulder));
            p.flags |= static_cast<unsigned short>(4u << hand);
        }
        float upm = 0.0f;
        if (ArmIk_GetUnitsPerMetre(hand, &upm))
            p.unitsPerMetre[hand] = upm;
    }
    return any;
}

// Everything that arrives is treated as hostile, because anything that can
// reach the port can send it (2026-09-19).
//
// The packet has no lengths, no counts, no offsets and no text in it - it is a
// fixed number of fixed-size floats, and one that is not exactly the right size
// has already been thrown away. So there is nothing here to overflow. What IS
// left is the values themselves: a NaN, an infinity or an absurd number would
// go straight into a skeleton once the arms are actually driven, and a NaN
// through an IK solve poisons everything it touches.
//
// Cheaper to refuse it at the door than to find out later which joint went to
// nowhere. A person is not three metres from their own head, and the rows of a
// rotation are unit length by definition.
bool Sane(const Packet& p)
{
    for (int hand = 0; hand < 2; ++hand) {
        // Only the hands the sender says are THERE (2026-09-20).
        //
        // This used to check both unconditionally, and a hand that is not
        // tracked is sent as zeros - so its basis rows have length 0, the unit
        // check below fails, and the WHOLE packet is thrown away including the
        // good hand. One controller glancing away from the cameras therefore
        // discarded the other, and nothing at all got through until both were
        // tracked.
        //
        // The co-op log said so plainly and I nearly missed it: "sent 26, 24
        // arrived, 0 used in the last second" - a full stream landing and being
        // binned. That is what made "Driving their arms" flicker, because with
        // no packets accepted the partner's hands went stale every half second
        // and the solve correctly declined.
        if (!(p.flags & (1u << hand)))
            continue;
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(p.handFromHead[hand][i]) || std::fabs(p.handFromHead[hand][i]) > 3.0f)
                return false;
            if (!std::isfinite(p.shoulderFromHead[hand][i]) || std::fabs(p.shoulderFromHead[hand][i]) > 1.5f)
                return false;
        }
        for (int i = 0; i < 9; ++i) {
            if (!std::isfinite(p.basis[hand][i]) || std::fabs(p.basis[hand][i]) > 1.01f)
                return false;
        }
        if (!std::isfinite(p.unitsPerMetre[hand]) || p.unitsPerMetre[hand] < 0.0f
            || p.unitsPerMetre[hand] > 1000.0f)
            return false;
        for (int r = 0; r < 3; ++r) {
            const float* row = p.basis[hand] + r * 3;
            const float len = row[0] * row[0] + row[1] * row[1] + row[2] * row[2];
            if (len < 0.9f || len > 1.1f)
                return false;
        }
    }
    return true;
}

// How long a hand keeps its last known pose after the sender stops describing
// it. A controller glancing away from the headset's cameras is a fraction of a
// second; putting one down is not. Two seconds holds through the first without
// leaving an arm stuck up in the air after the second.
constexpr unsigned long long kHandHoldMs = 2000;

// Merge a packet into what we know, rather than replace it (2026-09-20).
//
// Replacing was wrong in a way that only shows on a bad frame: a hand the
// sender could not see arrives as "not there", the solve skips that arm, and
// the game's animation takes it back for as long as the dropout lasts. One
// controller dipping behind the player's back would hand an arm to the
// animation and snatch it away again, which is precisely what a VR game must
// never do - "it has to be proper IK driving at all times".
//
// So a hand that stops being described keeps the last pose it had. The arm
// holds still for a moment instead of flicking back to whatever the animation
// wanted, which is both less noticeable and honest: we genuinely do not know
// where that hand is, and the last place we saw it is the best answer.
void MergeIntoPartner(const Packet& p)
{
    const unsigned long long now = GetTickCount64();
    for (int hand = 0; hand < 2; ++hand) {
        if (p.flags & (1u << hand)) {
            std::memcpy(g_partner.handFromHead[hand], p.handFromHead[hand],
                sizeof(g_partner.handFromHead[hand]));
            std::memcpy(g_partner.basis[hand], p.basis[hand], sizeof(g_partner.basis[hand]));
            g_partner.haveHand[hand] = true;
            g_partnerHandMs[hand] = now;
        } else if (g_partner.haveHand[hand] && now - g_partnerHandMs[hand] > kHandHoldMs) {
            g_partner.haveHand[hand] = false;
        }
        // The shoulder and the scale come from a T-pose, so they do not change
        // while somebody plays. Once known, they are kept.
        g_partner.twoHanded = (p.flags & 16u) != 0;
        if (p.flags & (4u << hand)) {
            std::memcpy(g_partner.shoulderFromHead[hand], p.shoulderFromHead[hand],
                sizeof(g_partner.shoulderFromHead[hand]));
            g_partner.haveShoulder[hand] = true;
        }
        if (p.unitsPerMetre[hand] > 1.0f)
            g_partner.unitsPerMetre[hand] = p.unitsPerMetre[hand];
    }
    g_partner.ms = now;
}

void TakePacket(const Packet& p, const sockaddr_in& from)
{
    AcquireSRWLockExclusive(&g_lock);
    MergeIntoPartner(p);
    g_havePartner = true;
    // Answer whoever is talking to us, if nobody was named in the settings.
    if (!g_havePeer) {
        g_peer = from;
        g_havePeer = true;
        g_peerLearned = true;
    }
    ReleaseSRWLockExclusive(&g_lock);
}

DWORD WINAPI Worker(LPVOID)
{
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        SetStatus("could not start Winsock");
        Log_Printf("IkSync: WSAStartup failed");
        return 0;
    }

    IkSyncSettings settings;
    AcquireSRWLockShared(&g_lock);
    settings = g_settings;
    ReleaseSRWLockShared(&g_lock);

    const SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        SetStatus("could not open a socket");
        Log_Printf("IkSync: socket() failed (%d)", WSAGetLastError());
        WSACleanup();
        return 0;
    }

    sockaddr_in bindTo = {};
    bindTo.sin_family = AF_INET;
    bindTo.sin_addr.s_addr = htonl(INADDR_ANY);
    bindTo.sin_port = htons(static_cast<unsigned short>(settings.port));
    if (bind(sock, reinterpret_cast<sockaddr*>(&bindTo), sizeof(bindTo)) == SOCKET_ERROR) {
        char msg[128];
        _snprintf_s(msg, sizeof(msg), _TRUNCATE, "port %d is busy - pick another on BOTH machines",
            settings.port);
        SetStatus(msg);
        // Not retried on the next port up, on purpose. A typed address is sent
        // to the configured port, so an end that quietly moved would be
        // listening somewhere the other end is not talking, which is a far
        // more confusing failure than being told the port is taken.
        Log_Printf("IkSync: could not bind port %d (error %d). Something else has it - Steam uses 27000 "
                   "to 27100, so avoid those. Pick another port, the same one on both machines.",
            settings.port, WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 0;
    }

    // If an address was typed in, that is the peer and a received packet does
    // not get to change it.
    AcquireSRWLockExclusive(&g_lock);
    g_havePeer = false;
    g_peerLearned = false;
    if (settings.partnerIp[0]) {
        in_addr addr = {};
        if (inet_pton(AF_INET, settings.partnerIp, &addr) == 1) {
            g_peer = {};
            g_peer.sin_family = AF_INET;
            g_peer.sin_addr = addr;
            g_peer.sin_port = htons(static_cast<unsigned short>(settings.port));
            g_havePeer = true;
        } else {
            Log_Printf("IkSync: '%s' is not an address I can read", settings.partnerIp);
        }
    }
    ReleaseSRWLockExclusive(&g_lock);

    Log_Printf("IkSync: listening on port %d%s%s", settings.port, settings.partnerIp[0] ? ", talking to " : "",
        settings.partnerIp[0] ? settings.partnerIp : "");
    LogLoadedModules("arm sharing started");
    ProbeSteam();
    WatchSteamPartner();

    // Watch the game's own traffic, unless an address was typed in. The typed
    // one always wins: it is the way out when the game turns out to be relayed.
    g_ourPortNet = htons(static_cast<unsigned short>(settings.port));
    if (!settings.partnerIp[0]) {
        InstallWatch();
        InterlockedExchange(&g_watching, 1);
    }

    unsigned seq = 0;
    double lastSendMs = 0.0;
    unsigned long long lastFindMs = 0;
    unsigned long long lastModulesMs = 0;
    unsigned long long lastReportMs = GetTickCount64();
    unsigned long long sentSince = 0, recvSince = 0;
    // Every datagram that lands, whatever it turns out to be. Silence and
    // rubbish are different faults and the first co-op test could not tell
    // them apart (2026-09-19): one means the packets never arrive, the other
    // means they arrive and something is wrong with them.
    unsigned long long rawSince = 0, rejectSize = 0, rejectMagic = 0, rejectVersion = 0, rejectSane = 0;
    bool hadHands = false;

    while (InterlockedCompareExchange(&g_stop, 0, 0) == 0) {
        // Drain whatever has arrived. Non-blocking by way of select, so the
        // loop always comes back round to the send and to the stop flag.
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(sock, &readable);
        timeval wait = { 0, 5000 }; // 5 ms
        if (select(0, &readable, nullptr, nullptr, &wait) > 0) {
            // Bounded, so a flood cannot hold this loop open and starve the
            // send and the stop flag.
            for (int drained = 0; drained < 64; ++drained) {
                Packet in = {};
                sockaddr_in from = {};
                int fromLen = sizeof(from);
                const int got = recvfrom(sock, reinterpret_cast<char*>(&in), sizeof(in), 0,
                    reinterpret_cast<sockaddr*>(&from), &fromLen);
                if (got < 0) {
                    // Too big to be ours, but it HAS been taken off the queue.
                    // Carrying on is what keeps oversized rubbish from hiding
                    // the packets behind it.
                    if (WSAGetLastError() == WSAEMSGSIZE)
                        continue;
                    break;
                }
                if (got == 0)
                    break;
                ++rawSince;
                if (got != static_cast<int>(sizeof(in))) {
                    ++rejectSize;
                    continue;
                }
                if (in.magic != kMagic) {
                    ++rejectMagic;
                    continue; // somebody else's traffic on our port
                }
                if (in.version != kVersion) {
                    ++rejectVersion;
                    static bool s_said = false;
                    if (!s_said) {
                        s_said = true;
                        Log_Printf("IkSync: the other end is speaking version %u and this is %u. "
                                   "You both need the same build.",
                            in.version, kVersion);
                    }
                    continue;
                }
                if (!Sane(in)) {
                    ++rejectSane;
                    static unsigned long long s_saidAt = 0;
                    const unsigned long long nowBad = GetTickCount64();
                    if (nowBad - s_saidAt > 5000) {
                        s_saidAt = nowBad;
                        Log_Printf("IkSync: threw away a packet with impossible numbers in it");
                    }
                    continue;
                }
                TakePacket(in, from);
                ++recvSince;
            }
        }

        const unsigned long long now = GetTickCount64();

        // Anything the game has loaded since the last look. This is how the
        // module that does the co-op networking gets named: it will be loaded
        // when you go online, not when the game starts, and not when this
        // setting is switched on either, since a saved setting switches itself
        // on at launch.
        if (now - lastModulesMs >= 5000) {
            lastModulesMs = now;
            LogNewModules();
        }

        // Nobody to talk to yet, and nobody has talked to us: ask who the game
        // itself is talking to. Checked once a second rather than every pass,
        // and only until it has an answer.
        if (InterlockedCompareExchange(&g_watching, 1, 1) == 1 && now - lastFindMs >= 1000) {
            lastFindMs = now;
            bool havePeer;
            AcquireSRWLockShared(&g_lock);
            havePeer = g_havePeer;
            ReleaseSRWLockShared(&g_lock);
            // Say what has been seen while the search is still going, every
            // five seconds. A search that finds nothing has to report what it
            // looked at, or the next session is as blind as the last one.
            static unsigned long long s_reportedAt = 0;
            if (!havePeer && now - s_reportedAt >= 5000) {
                s_reportedAt = now;
                ReportWatch();
                // And the answer we actually expect to get, since the game
                // talks to its partner through Steam rather than through a
                // socket. This is the line that says whether the whole Steam
                // route is real.
                static unsigned long long s_saidPartner = 0;
                const unsigned long long partner = PartnerSteamID();
                if (partner && partner != s_saidPartner) {
                    s_saidPartner = partner;
                    Log_Printf("SteamP2P: the game is talking to SteamID %llu (%ld calls so far). That is "
                               "the partner, and no address was needed to find them.",
                        partner, g_p2pCalls);
                } else if (!partner) {
                    Log_Printf("SteamP2P: %ld SendP2PPacket calls so far, no partner yet", g_p2pCalls);
                }
            }
            unsigned long found = 0;
            char why[160];
            if (!havePeer && BestCandidate(&found, why, sizeof(why))) {
                AcquireSRWLockExclusive(&g_lock);
                g_peer = {};
                g_peer.sin_family = AF_INET;
                g_peer.sin_addr.s_addr = found;
                g_peer.sin_port = htons(static_cast<unsigned short>(settings.port));
                g_havePeer = true;
                g_peerLearned = true;
                ReleaseSRWLockExclusive(&g_lock);
                Log_Printf("IkSync: the game is talking to %s, so that is where the arms go. If nothing "
                           "comes back, this is a relay rather than your partner and the address has to "
                           "be typed in.",
                    why);
            }
        }

        // Paced off the performance counter, not GetTickCount64 (2026-09-19).
        // That clock moves in steps of about 15.6 ms, so a 33 ms gate does not
        // fire at 33 ms, it fires at 46.8 - which is exactly the 21 a second
        // the first co-op test reported when it should have been 30.
        if (NowMs() - lastSendMs >= 1000 / kSendHz) {
            lastSendMs = NowMs();
            sockaddr_in peer;
            bool havePeer;
            AcquireSRWLockShared(&g_lock);
            peer = g_peer;
            havePeer = g_havePeer;
            ReleaseSRWLockShared(&g_lock);
            if (havePeer) {
                Packet out = {};
                out.magic = kMagic;
                out.version = kVersion;
                out.seq = ++seq;
                // Sent even when there is nothing to say (2026-09-19). A
                // flatscreen player has no hands to send - their arms are the
                // animation, which the game already syncs - but they still have
                // to send SOMETHING, and for two reasons that both matter.
                //
                // A packet from them is how the VR player's end learns their
                // address when discovery has not found it. And a hole in a NAT
                // is only open in the direction somebody has sent through it,
                // so an end that never sends can never be reached. Stay silent
                // and the one case we most want - one player in a headset, one
                // on a pad - is the case that cannot connect.
                //
                // An empty packet is 132 bytes at 30 Hz. Nothing.
                BuildLocal(out);
                sendto(sock, reinterpret_cast<const char*>(&out), sizeof(out), 0,
                    reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));
                ++sentSince;
                if (out.flags)
                    hadHands = true;
            }
        }

        // A line a second while anything is moving, because the whole point of
        // the first co-op test is to find out whether the packets arrive at all.
        if (now - lastReportMs >= 1000) {
            const float secs = (now - lastReportMs) / 1000.0f;
            lastReportMs = now;
            bool havePeer, learned, have;
            sockaddr_in peer;
            IkSyncHands hands;
            AcquireSRWLockShared(&g_lock);
            havePeer = g_havePeer;
            learned = g_peerLearned;
            peer = g_peer;
            have = g_havePartner;
            hands = g_partner;
            ReleaseSRWLockShared(&g_lock);

            char addr[64] = "nobody";
            if (havePeer)
                inet_ntop(AF_INET, &peer.sin_addr, addr, sizeof(addr));

            char status[160];
            if (!havePeer) {
                strcpy_s(status, "waiting to be spoken to");
            } else if (recvSince == 0) {
                _snprintf_s(status, sizeof(status), _TRUNCATE, "sending to %s, nothing back", addr);
            } else {
                // Not being in VR is not a fault, so it does not read like one:
                // a flatscreen player is meant to be listening only.
                _snprintf_s(status, sizeof(status), _TRUNCATE, "%s, %.0f a second%s", addr,
                    recvSince / secs, hadHands ? "" : " (listening only, no hands here)");
            }
            hadHands = false;
            SetStatus(status);

            // Say so when things land and are then thrown away. This is the
            // line that tells a blocked path apart from a broken packet.
            if (rawSince != recvSince) {
                Log_Printf("IkSync: %llu datagrams arrived and %llu were used - %llu wrong size, %llu not "
                           "ours, %llu wrong version, %llu impossible numbers",
                    rawSince, recvSince, rejectSize, rejectMagic, rejectVersion, rejectSane);
            }
            rawSince = rejectSize = rejectMagic = rejectVersion = rejectSane = 0;

            if (sentSince || recvSince) {
                if (have && recvSince) {
                    Log_Printf("IkSync: %s%s | sent %llu, got %llu in the last second | their right hand "
                               "%+.2f %+.2f %+.2f",
                        addr, learned ? " (learned)" : "", sentSince, recvSince, hands.handFromHead[1][0],
                        hands.handFromHead[1][1], hands.handFromHead[1][2]);
                } else {
                    Log_Printf("IkSync: %s | sent %llu, got %llu in the last second", addr, sentSince,
                        recvSince);
                }
            }
            sentSince = recvSince = 0;
        }
    }

    closesocket(sock);
    WSACleanup();
    Log_Printf("IkSync: stopped");
    return 0;
}

} // namespace

void IkSync_Pump()
{
    // Steam's own peer-to-peer, which is what this should have been from the
    // start (2026-09-19). Addressed by SteamID, so there is no address to type,
    // no port to forward and no NAT to punch: Valve already solved all of it,
    // and relays the traffic themselves when a direct route cannot be had.
    //
    // Called once a frame from the camera hook, which means every Steam call
    // here happens on the game's own thread, the same thread the game uses.
    // Doing it from the socket worker would be asking two threads into an
    // interface with no promise of being safe for that.
    IkSyncSettings settings;
    AcquireSRWLockShared(&g_lock);
    settings = g_settings;
    ReleaseSRWLockShared(&g_lock);
    if (!settings.enabled)
        return;

    // Talking to yourself, for testing. Before the Steam guard on purpose:
    // there is no partner, no session and possibly no Steam at all, and none
    // of that is what is being tested.
    if (settings.loopback) {
        // Room for the higher rate: at ninety a second a tenth of a second of
        // delay is nine packets, and eight slots would have started throwing
        // them away before they were due.
        static Packet s_held[32] = {};
        static double s_heldAt[32] = {};
        static int s_next = 0;
        static unsigned s_loopSeq = 0;
        static double s_lastLoopMs = 0.0;
        const double loopNow = NowMs();
        if (loopNow - s_lastLoopMs >= 1000.0 / kSendHz) {
            s_lastLoopMs = loopNow;
            Packet mine = {};
            mine.magic = kMagic;
            mine.version = kVersion;
            mine.seq = ++s_loopSeq;
            BuildLocal(mine);
            s_held[s_next] = mine;
            s_heldAt[s_next] = loopNow;
            s_next = (s_next + 1) % 32;
        }
        // A tenth of a second late, the way a network would.
        for (int i = 0; i < 32; ++i) {
            if (s_heldAt[i] <= 0.0 || loopNow - s_heldAt[i] < 100.0)
                continue;
            Packet in = s_held[i];
            s_heldAt[i] = 0.0;
            if (in.magic != kMagic || in.version != kVersion || !Sane(in))
                continue;
            AcquireSRWLockExclusive(&g_lock);
            MergeIntoPartner(in);
            g_havePartner = true;
            ReleaseSRWLockExclusive(&g_lock);
        }
        static double s_toldLoopMs = 0.0;
        if (loopNow - s_toldLoopMs >= 3000.0) {
            s_toldLoopMs = loopNow;
            Log_Printf("IkSync: talking to myself - my own hands are being fed back in as a partner's, a tenth of "
                       "a second late. Nothing is on the wire.");
        }
        return;
    }

    if (!g_steamNet || !g_origSendP2P || !g_readP2P || !g_isP2PAvail)
        return;

    const unsigned long long partner = PartnerSteamID();
    if (!partner)
        return;

    // Accept once. The game already holds a session with this player, so this
    // is belt and braces rather than a requirement - but a session that was
    // never accepted silently drops everything, which would be a maddening
    // thing to debug later.
    static unsigned long long s_accepted = 0;
    if (g_acceptP2P && s_accepted != partner) {
        s_accepted = partner;
        g_acceptP2P(g_steamNet, nullptr, partner);
        Log_Printf("SteamP2P: sending arms to SteamID %llu on channel %d", partner, kArmChannel);
    }

    static double s_lastSendMs = 0.0;
    const double nowMs = NowMs();
    if (nowMs - s_lastSendMs >= 1000.0 / kSendHz) {
        s_lastSendMs = nowMs;
        Packet out = {};
        out.magic = kMagic;
        out.version = kVersion;
        static unsigned s_seq = 0;
        out.seq = ++s_seq;
        BuildLocal(out); // sent even with nothing in it, exactly as over UDP
        // Straight to the original, not through our own watcher: what we send
        // is not evidence about who the partner is.
        g_origSendP2P(g_steamNet, nullptr, partner, &out, sizeof(out), 0 /* unreliable */, kArmChannel);
        ++g_steamSent;
    }

    // Drain our channel. Bounded, so a flood cannot hold a frame open.
    for (int drained = 0; drained < 16; ++drained) {
        unsigned waiting = 0;
        if (!g_isP2PAvail(g_steamNet, nullptr, &waiting, kArmChannel))
            break;
        Packet in = {};
        unsigned got = 0;
        unsigned long long from = 0;
        if (!g_readP2P(g_steamNet, nullptr, &in, sizeof(in), &got, &from, kArmChannel))
            break;
        ++g_steamRaw;
        // Each refusal counted separately (2026-09-20). This was one combined
        // test that said nothing, and a whole session was spent not knowing
        // that a full stream was arriving and being discarded. "Arrived but
        // not used" is four completely different faults wearing one face.
        if (got != static_cast<unsigned>(sizeof(in))) {
            ++g_steamBadSize;
            continue;
        }
        if (in.magic != kMagic) {
            ++g_steamNotOurs;
            continue;
        }
        if (in.version != kVersion) {
            ++g_steamBadVersion;
            continue;
        }
        if (!Sane(in)) {
            ++g_steamInsane;
            continue;
        }
        PinPartner(from);
        AcquireSRWLockExclusive(&g_lock);
        MergeIntoPartner(in);
        g_havePartner = true;
        ReleaseSRWLockExclusive(&g_lock);
        ++g_steamGot;
    }

    // One line a second, and only while something is happening.
    static double s_saidAtMs = 0.0;
    if (nowMs - s_saidAtMs >= 1000.0) {
        s_saidAtMs = nowMs;
        if (g_steamSent || g_steamRaw) {
            char status[160];
            _snprintf_s(status, sizeof(status), _TRUNCATE, "Steam %llu, sent %u, got %u a second", partner,
                g_steamSent, g_steamGot);
            SetStatus(status);
            Log_Printf("SteamP2P: sent %u, %u arrived, %u used in the last second", g_steamSent, g_steamRaw,
                g_steamGot);
            if (g_steamRaw != g_steamGot) {
                Log_Printf("SteamP2P: threw away %u wrong size, %u not ours, %u wrong version, %u with "
                           "impossible numbers",
                    g_steamBadSize, g_steamNotOurs, g_steamBadVersion, g_steamInsane);
            }
        }
        InterlockedExchange(&g_steamSentLast, static_cast<LONG>(g_steamSent));
        InterlockedExchange(&g_steamGotLast, static_cast<LONG>(g_steamGot));
        g_steamSent = g_steamRaw = g_steamGot = 0;
        g_steamBadSize = g_steamNotOurs = g_steamBadVersion = g_steamInsane = 0;
    }
}

void IkSync_SetSettings(const IkSyncSettings& s)
{
    AcquireSRWLockExclusive(&g_lock);
    const bool changed = g_settings.enabled != s.enabled || g_settings.port != s.port
        || strcmp(g_settings.partnerIp, s.partnerIp) != 0;
    g_settings = s;
    if (changed)
        g_wantRunning = false; // forces a restart on the next Update
    ReleaseSRWLockExclusive(&g_lock);
}

IkSyncSettings IkSync_GetSettings()
{
    AcquireSRWLockShared(&g_lock);
    const IkSyncSettings s = g_settings;
    ReleaseSRWLockShared(&g_lock);
    return s;
}

void IkSync_Update()
{
    bool want;
    AcquireSRWLockShared(&g_lock);
    want = g_settings.enabled;
    const bool running = g_wantRunning;
    ReleaseSRWLockShared(&g_lock);

    if (want == running && (!want || g_thread))
        return;

    IkSync_Shutdown();
    if (!want) {
        SetStatus("off");
        return;
    }

    InterlockedExchange(&g_stop, 0);
    AcquireSRWLockExclusive(&g_lock);
    g_wantRunning = true;
    g_havePartner = false;
    ReleaseSRWLockExclusive(&g_lock);
    SetStatus("starting");
    g_thread = CreateThread(nullptr, 0, Worker, nullptr, 0, nullptr);
    if (!g_thread) {
        SetStatus("could not start");
        AcquireSRWLockExclusive(&g_lock);
        g_wantRunning = false;
        ReleaseSRWLockExclusive(&g_lock);
    }
}

void IkSync_Shutdown()
{
    if (!g_thread)
        return;
    InterlockedExchange(&g_stop, 1);
    // The worker never waits longer than its select timeout, so this is short.
    WaitForSingleObject(g_thread, 2000);
    CloseHandle(g_thread);
    g_thread = nullptr;
    AcquireSRWLockExclusive(&g_lock);
    g_wantRunning = false;
    g_havePartner = false;
    ReleaseSRWLockExclusive(&g_lock);
}

bool IkSync_GetPartnerHands(IkSyncHands& out)
{
    AcquireSRWLockShared(&g_lock);
    const bool have = g_havePartner && GetTickCount64() - g_partner.ms < kStaleMs;
    if (have)
        out = g_partner;
    ReleaseSRWLockShared(&g_lock);
    return have;
}

bool IkSync_GetSteamStatus(unsigned long long* partner, unsigned* sentPerSecond, unsigned* gotPerSecond)
{
    if (!g_steamNet || !g_origSendP2P)
        return false;
    const unsigned long long id = PartnerSteamID();
    if (partner)
        *partner = id;
    if (sentPerSecond)
        *sentPerSecond = static_cast<unsigned>(InterlockedCompareExchange(&g_steamSentLast, 0, 0));
    if (gotPerSecond)
        *gotPerSecond = static_cast<unsigned>(InterlockedCompareExchange(&g_steamGotLast, 0, 0));
    return true;
}

void IkSync_DescribeStatus(char* out, size_t size)
{
    if (!out || !size)
        return;
    AcquireSRWLockShared(&g_lock);
    strncpy_s(out, size, g_status, _TRUNCATE);
    ReleaseSRWLockShared(&g_lock);
}
