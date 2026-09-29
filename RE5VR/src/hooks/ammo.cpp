#include "ammo.h"

#include "aim_finder.h"

#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>

#include <cstdio>
#include <cstring>

// See ammo.h for how the address was found and why it is a hook rather than a
// pointer path.

namespace {

// The call that follows "mov [esi+1Ch], edx". Five bytes, a whole
// instruction, and one the watchpoint proved runs on every ammunition write.
constexpr unsigned long kCallRva = 0x5EDD08;
constexpr int kStoreBack = 3; // the store begins three bytes earlier
constexpr unsigned char kExpected[] = { 0x89, 0x56, 0x1C, 0xE8, 0xF3, 0xE4, 0x25, 0x00 };

constexpr unsigned kOffState = 0x18;
constexpr unsigned kOffLoaded = 0x1C;
constexpr unsigned kOffMark = 0x24;
// WHICH WEAPON THIS IS (2026-09-26). The mark at +24h reads 0201 on every
// weapon in the game, so it is a kind rather than a name, and the low byte
// being 1 on the M92F - whose weapon id is also 1 - was a coincidence that
// cost an hour. This one read 1 on the same weapon in the same dump and is the
// candidate that has not been ruled out.
constexpr unsigned kOffId = 0x14;
constexpr unsigned kOffCapacity = 0x2C;
constexpr unsigned kOffReserve = 0x34;

// WHERE THE COUNT IS COPIED FROM (2026-09-25, corrected).
//
// The store the watchpoint caught reads its value two instructions earlier:
//
//     8B 57 08    mov edx,[edi+8]
//     89 56 1C    mov [esi+1Ch], edx
//
// So the source is edi+8 and nothing else. The first version latched ebx and
// wrote ebx+0x9C, because in the session where this was found edi happened to
// sit 0x94 past ebx and an address at ebx+0x9C had survived every narrowing.
// That offset is where one item's record landed that once, not a rule, and the
// guard caught it the moment it was wrong: "the record at 0BFCE720+9C holds 29,
// not the 10 in the gun - left alone", a hundred times over, with the eject
// firing correctly each time and the game restoring the count a frame later.
constexpr unsigned kOffOwnerLoaded = 0x8;

constexpr int kMaxSlots = 24;

struct Slot {
    volatile LONG mag;   // esi, 0 while the slot is free
    volatile LONG owner; // ebx
    volatile LONG hits;
    int loaded, capacity, reserve;
    unsigned mark, state, id;
    unsigned long long lastMs; // when loaded last changed
    bool alive;
    bool everRead;
};

Slot g_slots[kMaxSlots];
volatile LONG g_overflow = 0;
volatile LONG g_installed = 0;
void* g_trampoline = nullptr;

bool TryRead(const void* at, void* into, unsigned bytes)
{
    if (!at)
        return false;
    __try {
        std::memcpy(into, at, bytes);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool TryWriteInt(void* at, int value)
{
    if (!at)
        return false;
    __try {
        *static_cast<volatile int*>(at) = value;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

unsigned char* AsPointer(LONG v)
{
    return reinterpret_cast<unsigned char*>(static_cast<uintptr_t>(static_cast<unsigned long>(v)));
}

int SlotFor(unsigned char* mag)
{
    const LONG m = static_cast<LONG>(reinterpret_cast<uintptr_t>(mag));
    for (int i = 0; i < kMaxSlots; ++i)
        if (g_slots[i].mag == m)
            return i;
    return -1;
}

} // namespace

// Called from the stub with the two registers the write left behind: the
// weapon in esi, and in edi the record its count was just copied out of. Runs on
// whichever thread the game happens to be on, several times a second, so it
// takes no lock, allocates nothing and says nothing.
extern "C" void __cdecl Ammo_Seen(unsigned char* mag, unsigned char* owner)
{
    const LONG m = static_cast<LONG>(reinterpret_cast<uintptr_t>(mag));
    if (!m)
        return;
    for (int i = 0; i < kMaxSlots; ++i) {
        LONG cur = g_slots[i].mag;
        if (cur == 0) {
            const LONG prev = InterlockedCompareExchange(&g_slots[i].mag, m, 0);
            cur = (prev == 0) ? m : prev;
        }
        if (cur == m) {
            InterlockedExchange(&g_slots[i].owner, static_cast<LONG>(reinterpret_cast<uintptr_t>(owner)));
            InterlockedIncrement(&g_slots[i].hits);
            return;
        }
    }
    InterlockedIncrement(&g_overflow);
}

namespace {

__declspec(naked) void AmmoHook_Stub()
{
    __asm {
        pushad
        pushfd
        mov ebp, esp
        sub esp, 512
        and esp, 0FFFFFFF0h
        fxsave [esp]
        // edi, not ebx: the count is read from [edi+8], and that is the copy a
        // reload has to change. ebx is the object the call after this one is
        // made on, which is a different thing entirely.
        push edi
        push esi
        call Ammo_Seen
        add esp, 8
        fxrstor [esp]
        mov esp, ebp
        popfd
        popad
        jmp g_trampoline
    }
}

} // namespace

void Ammo_Install()
{
    if (g_installed)
        return;
    InterlockedExchange(&g_installed, -1); // tried, and only once

    unsigned char* exe = reinterpret_cast<unsigned char*>(GetModuleHandleA(nullptr));
    unsigned char* store = exe + kCallRva - kStoreBack;
    unsigned char seen[sizeof(kExpected)] = {};
    if (!TryRead(store, seen, sizeof(seen))) {
        Log_Printf("Ammo: re5dx9.exe+%lX could not be read - no magazine hook", kCallRva - kStoreBack);
        return;
    }
    if (std::memcmp(seen, kExpected, sizeof(kExpected)) != 0) {
        char hex[sizeof(kExpected) * 3 + 1] = {};
        for (int i = 0; i < static_cast<int>(sizeof(seen)); ++i)
            sprintf_s(hex + i * 3, 4, "%02X ", seen[i]);
        Log_Printf("Ammo: re5dx9.exe+%lX holds %s- not the ammunition store this was found on, so nothing is hooked",
            kCallRva - kStoreBack, hex);
        return;
    }

    void* target = exe + kCallRva;
    MH_STATUS st = MH_CreateHook(target, reinterpret_cast<void*>(&AmmoHook_Stub), &g_trampoline);
    if (st == MH_OK)
        st = MH_EnableHook(target);
    if (st != MH_OK) {
        Log_Printf("Ammo: the hook at re5dx9.exe+%lX would not go on (%d)", kCallRva, static_cast<int>(st));
        return;
    }
    InterlockedExchange(&g_installed, 1);
    Log_Printf("Ammo: watching the magazine from re5dx9.exe+%lX - rounds at +1Ch, capacity at +2Ch, spare at +34h",
        kCallRva);
}

bool Ammo_Installed()
{
    return g_installed == 1;
}

void Ammo_OnEndScene()
{
    if (g_installed != 1)
        return;
    const unsigned long long now = GetTickCount64();
    for (int i = 0; i < kMaxSlots; ++i) {
        Slot& s = g_slots[i];
        if (!s.mag)
            continue;
        unsigned char block[0x40] = {};
        if (!TryRead(AsPointer(s.mag), block, sizeof(block))) {
            s.alive = false;
            continue;
        }
        s.alive = true;
        const int loaded = *reinterpret_cast<const int*>(block + kOffLoaded);
        if (!s.everRead || loaded != s.loaded) {
            s.lastMs = now;
            s.everRead = true;
        }
        s.loaded = loaded;
        s.capacity = *reinterpret_cast<const int*>(block + kOffCapacity);
        s.reserve = *reinterpret_cast<const int*>(block + kOffReserve);
        s.mark = *reinterpret_cast<const unsigned*>(block + kOffMark);
        s.id = *reinterpret_cast<const unsigned*>(block + kOffId);
        s.state = *reinterpret_cast<const unsigned*>(block + kOffState);
    }
}

int Ammo_Slots(AmmoSlot* out, int max)
{
    int n = 0;
    for (int i = 0; i < kMaxSlots && n < max; ++i) {
        const Slot& s = g_slots[i];
        if (!s.mag)
            continue;
        AmmoSlot& o = out[n++];
        o.mag = AsPointer(s.mag);
        o.owner = AsPointer(s.owner);
        o.hits = static_cast<unsigned long>(s.hits);
        o.loaded = s.loaded;
        o.capacity = s.capacity;
        o.reserve = s.reserve;
        o.mark = s.mark;
        o.id = s.id;
        o.state = s.state;
        o.lastMs = s.lastMs;
        o.alive = s.alive;
    }
    // Busiest first: the gun being fired is written far more often than a
    // magazine sitting in the case.
    for (int a = 0; a < n; ++a)
        for (int b = a + 1; b < n; ++b)
            if (out[b].hits > out[a].hits) {
                const AmmoSlot t = out[a];
                out[a] = out[b];
                out[b] = t;
            }
    return n;
}

unsigned g_preferId = 0;
bool g_havePreferId = false;

void Ammo_PreferWeaponId(unsigned id, bool have)
{
    g_preferId = id;
    g_havePreferId = have && id != 0;
}

bool Ammo_Held(AmmoSlot& out)
{
    AmmoSlot all[kMaxSlots];
    const int n = Ammo_Slots(all, kMaxSlots);
    const AmmoSlot* best = nullptr;
    // Two passes when a weapon is pinned: its own magazine first, and only if
    // nothing carries that id does the old "busiest with spare rounds" rule
    // get a say. See Ammo_PreferWeaponId.
    for (int pass = 0; pass < (g_havePreferId ? 2 : 1) && !best; ++pass) {
        const bool mineOnly = g_havePreferId && pass == 0;
    for (int i = 0; i < n; ++i) {
        const AmmoSlot& s = all[i];
        if (mineOnly && s.id != g_preferId)
            continue;
        // A magazine, rather than whatever else shares this code path: it has
        // to hold something, it cannot hold more than it holds, and nothing in
        // this game carries four figures of any one round.
        if (!s.alive || s.capacity <= 0 || s.capacity > 1000 || s.loaded < 0 || s.loaded > s.capacity
            || s.reserve < 0 || s.reserve > 100000)
            continue;
        // WHICH OF THEM IS YOURS (2026-09-26). The dump settled this: a pistol
        // showing 10 of 10 with 150 spare produced three candidates, and only
        // one of them was the weapon.
        //
        //   [0] 881 writes - 10 of 10,  150 spare   <- the magazine
        //   [1] 880 writes - 10 of 100,   0 spare   <- the same weapon's
        //   [2]   1 write  - 10 of 100,   0 spare      definition, twice over
        //
        // The impostors carry the M92F's MAXIMUM upgradeable capacity, 100,
        // rather than the 10 it currently holds, and they have no spare count
        // because a definition does not own ammunition. They track your shots
        // as faithfully as the real one, so "whichever moved last" cannot tell
        // them apart - and worse, it cannot recover, because zeroing a
        // magazine marks that slot as the most recent thing to move. One wrong
        // pick was permanent.
        //
        // A weapon you can reload has rounds to reload it with, and that is
        // what a definition never has. So the spare count leads, the number of
        // times the game has bothered to write it breaks ties, and the clock
        // is the last word rather than the first.
        bool better = false;
        if (!best)
            better = true;
        else if ((s.reserve > 0) != (best->reserve > 0))
            better = s.reserve > 0;
        else if (s.hits != best->hits)
            better = s.hits > best->hits;
        else
            better = s.lastMs > best->lastMs;
        if (better)
            best = &s;
    }
    }
    if (!best)
        return false;
    out = *best;
    return true;
}

void Ammo_Dump()
{
    if (g_installed != 1) {
        Log_Printf("Ammo: nothing hooked, so there is nothing to show");
        return;
    }
    AmmoSlot all[kMaxSlots];
    const int n = Ammo_Slots(all, kMaxSlots);
    Log_Printf("Ammo: %d magazine(s) seen, %ld that did not fit", n, g_overflow);
    for (int i = 0; i < n; ++i) {
        const AmmoSlot& s = all[i];
        Log_Printf("Ammo: [%d] %p from %p, written %lu time(s) - %d of %d, %d spare (mark %04X, state %08X)%s", i,
            s.mag, s.owner, s.hits, s.loaded, s.capacity, s.reserve, s.mark, s.state, s.alive ? "" : " - gone");
    }
    AmmoSlot held;
    if (!Ammo_Held(held)) {
        Log_Printf("Ammo: none of them looks like a loaded weapon just now");
        return;
    }
    Log_Printf("Ammo: the one in hand looks like %p, %d of %d with %d spare", held.mag, held.loaded, held.capacity,
        held.reserve);
    // Word by word, both structures. The weapon's own layout is known; the
    // record behind it is not, and the spare rounds have to be written there
    // too or the next sync undoes them.
    const struct {
        const char* what;
        unsigned char* at;
        int from, to;
    } windows[] = {
        { "weapon", held.mag, -0x20, 0x60 },
        { "record", held.owner, 0x00, 0x100 },
    };
    for (const auto& w : windows) {
        if (!w.at)
            continue;
        for (int off = w.from; off < w.to; off += 32) {
            unsigned v[8] = {};
            if (!TryRead(w.at + off, v, sizeof(v)))
                break;
            Log_Printf("Ammo:   %s%+05X: %08X %08X %08X %08X %08X %08X %08X %08X", w.what, off, v[0], v[1], v[2], v[3],
                v[4], v[5], v[6], v[7]);
        }
    }
}

bool Ammo_SetLoaded(int rounds)
{
    AmmoSlot s;
    if (!Ammo_Held(s) || rounds < 0 || rounds > s.capacity)
        return false;
    const int i = SlotFor(s.mag);
    bool ok = TryWriteInt(s.mag + kOffLoaded, rounds);
    // And the record it is refreshed from, or the copy above lasts one sync.
    //
    // Only where that record really is holding the same count, though. The
    // weapon's own offset is proven - a watchpoint caught the store landing on
    // it - and this one is inferred, from an address that survived the same
    // narrowing. Inferred is not good enough to write blind into a live
    // object, so it has to agree with what we already know before it is
    // touched, and a session where it never agrees simply never gets written.
    if (s.owner) {
        int mirror = 0;
        if (TryRead(s.owner + kOffOwnerLoaded, &mirror, sizeof(mirror)) && mirror == s.loaded)
            ok = TryWriteInt(s.owner + kOffOwnerLoaded, rounds) || ok;
        else if (mirror != rounds)
            Log_Printf("Ammo: the record at %p+%X holds %d, not the %d in the gun - left alone", s.owner,
                kOffOwnerLoaded, mirror, s.loaded);
    }
    if (ok && i >= 0) {
        g_slots[i].loaded = rounds;
        // NOT the clock. That field means "when the game last changed this",
        // and it is one of the things the picker ranks on, so bumping it here
        // let a magazine win the ranking by having been written BY US. Choosing
        // wrongly once then made that choice permanent.
    }
    return ok;
}

bool Ammo_SetReserve(int rounds)
{
    AmmoSlot s;
    if (!Ammo_Held(s) || rounds < 0)
        return false;
    const int i = SlotFor(s.mag);
    const bool ok = TryWriteInt(s.mag + kOffReserve, rounds);
    if (ok && i >= 0)
        g_slots[i].reserve = rounds;
    return ok;
}

void Ammo_FindTheBag()
{
    AmmoSlot s;
    if (!Ammo_Held(s)) {
        Log_Printf("Ammo: nothing in hand to follow");
        return;
    }
    Ammo_Dump();
    const int want = s.reserve;
    Log_Printf("Ammo: looking for %d spare round(s) - weapon %p, record %p", want, s.mag, s.owner);

    // 1. The number itself, anywhere in either structure. A count the game
    //    keeps twice is the pattern the rounds in the gun already followed.
    const struct {
        const char* what;
        unsigned char* at;
        int bytes;
    } here[] = {
        { "weapon", s.mag, 0x100 },
        { "record", s.owner, 0x400 },
    };
    for (const auto& h : here) {
        if (!h.at)
            continue;
        for (int off = 0; off + 4 <= h.bytes; off += 4) {
            int v = 0;
            if (!TryRead(h.at + off, &v, sizeof(v)))
                break;
            if (v == want)
                Log_Printf("Ammo:   %s+%03X holds %d", h.what, off, want);
        }
    }

    // 2. And one step further out. The inventory is its own thing, so what the
    //    record is most likely to hold is the way to it rather than the number.
    if (s.owner) {
        for (int off = 0; off + 4 <= 0x400; off += 4) {
            unsigned p = 0;
            if (!TryRead(s.owner + off, &p, sizeof(p)))
                break;
            if (p < 0x100000u || p > 0x7FFF0000u || (p & 3u))
                continue;
            unsigned char* out = reinterpret_cast<unsigned char*>(static_cast<uintptr_t>(p));
            for (int in = 0; in + 4 <= 0x100; in += 4) {
                int v = 0;
                if (!TryRead(out + in, &v, sizeof(v)))
                    break;
                if (v == want)
                    Log_Printf("Ammo:   record+%03X points at %08X, and %08X+%03X holds %d", off, p, p, in, want);
            }
        }
    }

    // 3. Whatever keeps putting the old number back. Same method that found
    //    the magazine, on the field that will not stay written.
    unsigned char* mirror = s.mag + kOffReserve;
    AimFinder_Rearm();
    AimFinder_Start(mirror, "the spare rounds");
    Log_Printf("Ammo: watching %p for four seconds - fire a shot or reload and it will say who refreshes it",
        mirror);
}

bool Ammo_Eject(bool keepTheRounds)
{
    AmmoSlot s;
    if (!Ammo_Held(s) || s.loaded <= 0)
        return false;
    // KEEPING THE ROUNDS IS NOT DONE HERE (2026-09-25, user: "I noticed that
    // the keep existing ammo checkbox does nothing, as the mag eject always
    // dumps you to 0 so the game always reloads a full mag and burns x amount
    // of ammo").
    //
    // Exactly so, and adding them to the spare count was never going to help:
    // that count is the mirror the game rewrites from an inventory we cannot
    // reach, so the addition lasted about a second and the reload billed for a
    // full magazine regardless.
    //
    // The rounds are kept somewhere else entirely, and without writing the
    // inventory at all. RE5's reload TOPS UP the magazine rather than filling
    // it from empty, so a magazine reading 8 when the reload fires costs two
    // rounds instead of ten. The gun stays at zero for as long as you are
    // carrying, which is the whole point of doing this by hand, and the true
    // count goes back for the one frame before the game is asked to reload.
    // See the reload gesture in xr_input.cpp.
    (void)keepTheRounds;
    const bool ok = Ammo_SetLoaded(0);
    Log_Printf("Ammo: magazine out with %d round(s) still in it", s.loaded);
    return ok;
}

bool Ammo_Seat(int* tookOut)
{
    AmmoSlot s;
    if (!Ammo_Held(s))
        return false;
    const int room = s.capacity - s.loaded;
    if (room <= 0 || s.reserve <= 0)
        return false;
    // As much as the gun holds, or everything left, whichever runs out first.
    const int take = room < s.reserve ? room : s.reserve;
    if (!Ammo_SetReserve(s.reserve - take))
        return false;
    if (!Ammo_SetLoaded(s.loaded + take))
        return false;
    if (tookOut)
        *tookOut = take;
    Log_Printf("Ammo: magazine in, %d round(s) taken, %d left in the bag", take, s.reserve - take);
    return true;
}
