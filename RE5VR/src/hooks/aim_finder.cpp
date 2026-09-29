#include "aim_finder.h"

#include "../util/log.h"

#include <windows.h>
#include <tlhelp32.h>

#include <atomic>
#include <cstdio>
#include <cstring>

// See aim_finder.h for why this exists. The mechanics follow boom_finder.cpp,
// which did the same job for the camera boom: debug registers, a vectored
// handler that may not log or allocate, and a worker thread to do the
// suspending, because a thread cannot suspend itself.

namespace {

constexpr int kMaxHits = 64;
constexpr DWORD kWindowMs = 4000;
constexpr DWORD kTrapFlag = 0x100;

struct HitEntry {
    volatile LONG eip; // 0 = free slot
    volatile LONG count;
    volatile LONG regsValid;
    volatile LONG slots; // which of the four watched addresses it touched
    DWORD eax, ecx, edx, ebx, esp, ebp, esi, edi; // at first hit
};

HitEntry g_hits[kMaxHits];
volatile LONG g_hitOverflow = 0;
volatile LONG g_recording = 0;
bool g_everArmed = false;

std::atomic<bool> g_running{ false };
std::atomic<bool> g_done{ false };
std::atomic<bool> g_reported{ false };
constexpr int kMaxWatch = 4;
void* g_watchAddress[kMaxWatch] = {};
char g_watchName[kMaxWatch][80] = {};
int g_watchCount = 0;
bool g_watchReads = false;
char g_watchWhat[64] = "";
PVOID g_handler = nullptr;

LONG CALLBACK WatchpointHandler(EXCEPTION_POINTERS* ep)
{
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP)
        return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* ctx = ep->ContextRecord;
    const LONG fired = static_cast<LONG>(ctx->Dr6 & 0xF);
    if (!fired) {
        // A trap already in flight while the watch is being cleared can
        // arrive with its status gone. If we ever armed and this isn't a
        // trace step, it is one of ours and must not reach the game.
        if (g_everArmed && !(ctx->EFlags & kTrapFlag))
            return EXCEPTION_CONTINUE_EXECUTION;
        return EXCEPTION_CONTINUE_SEARCH;
    }

    // Data breakpoints are traps: the address is the instruction AFTER the
    // one that wrote.
    if (g_recording) {
        const LONG eip = static_cast<LONG>(reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress));
        bool recorded = false;
        for (int i = 0; i < kMaxHits && !recorded; ++i) {
            LONG cur = g_hits[i].eip;
            if (cur == 0 && InterlockedCompareExchange(&g_hits[i].eip, eip, 0) == 0) {
                HitEntry& h = g_hits[i];
                h.eax = ctx->Eax;
                h.ecx = ctx->Ecx;
                h.edx = ctx->Edx;
                h.ebx = ctx->Ebx;
                h.esp = ctx->Esp;
                h.ebp = ctx->Ebp;
                h.esi = ctx->Esi;
                h.edi = ctx->Edi;
                InterlockedExchange(&h.regsValid, 1);
                cur = eip;
            } else {
                cur = g_hits[i].eip;
            }
            if (cur == eip) {
                InterlockedIncrement(&g_hits[i].count);
                // Which of the four addresses this instruction touched. An
                // instruction can reach more than one over a window, so these
                // accumulate rather than replace.
                InterlockedOr(&g_hits[i].slots, fired);
                recorded = true;
            }
        }
        if (!recorded)
            InterlockedIncrement(&g_hitOverflow);
    }

    ctx->Dr6 = 0;
    return EXCEPTION_CONTINUE_EXECUTION;
}

// Arms (address != 0) or clears (address == 0) a write watchpoint on every
// thread but this one. Must run on a worker thread. Nothing between Suspend
// and Resume may take a lock: the suspended thread might hold it.
void ApplyToAllThreads(bool arm, int* outArmed, int* outFailed)
{
    *outArmed = 0;
    *outFailed = 0;

    DWORD addr[kMaxWatch] = {};
    DWORD dr7 = 0;
    if (arm) {
        for (int s = 0; s < g_watchCount && s < kMaxWatch; ++s) {
            if (!g_watchAddress[s])
                continue;
            addr[s] = static_cast<DWORD>(reinterpret_cast<uintptr_t>(g_watchAddress[s]));
            dr7 |= 1u << (s * 2); // L0, L1, L2, L3: local enable
            // R/W: 01 breaks on a write, 11 on a read or a write. x86 has no
            // read-only condition, so finding a reader means taking the writers
            // with it and subtracting the ones already known (2026-09-17).
            dr7 |= (g_watchReads ? 3u : 1u) << (16 + s * 4);
            dr7 |= 3u << (18 + s * 4); // LEN = 11: four bytes
        }
    }

    const DWORD self = GetCurrentThreadId();
    const DWORD pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE)
        return;

    THREADENTRY32 te = {};
    te.dwSize = sizeof(te);
    for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self)
            continue;
        HANDLE t = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT | THREAD_SUSPEND_RESUME, FALSE, te.th32ThreadID);
        if (!t) {
            ++*outFailed;
            continue;
        }
        bool armed = false;
        if (SuspendThread(t) != static_cast<DWORD>(-1)) {
            CONTEXT c = {};
            c.ContextFlags = CONTEXT_DEBUG_REGISTERS;
            if (GetThreadContext(t, &c)) {
                c.Dr0 = addr[0];
                c.Dr1 = addr[1];
                c.Dr2 = addr[2];
                c.Dr3 = addr[3];
                // Dr6 left alone: clearing it under a trap mid-delivery hands
                // the handler an exception it cannot recognise.
                c.Dr7 = dr7;
                armed = SetThreadContext(t, &c) != FALSE;
            }
            ResumeThread(t);
        }
        if (armed)
            ++*outArmed;
        else
            ++*outFailed;
        CloseHandle(t);
    }
    CloseHandle(snap);
}

DWORD WINAPI WatchThread(LPVOID)
{
    int armed = 0, failed = 0;
    g_everArmed = true;
    InterlockedExchange(&g_recording, 1);
    ApplyToAllThreads(true, &armed, &failed);
    Log_Printf("AimFinder: watching %d address(es) for %lu ms - armed on %d thread(s), %d failed", g_watchCount,
        kWindowMs, armed, failed);
    for (int s = 0; s < g_watchCount; ++s)
        Log_Printf("AimFinder:   [%d] %p  %s", s, g_watchAddress[s], g_watchName[s]);

    Sleep(kWindowMs);

    InterlockedExchange(&g_recording, 0);
    ApplyToAllThreads(false, &armed, &failed);
    Log_Printf("AimFinder: window closed, watchpoints cleared on %d thread(s)", armed);
    g_done.store(true, std::memory_order_release);
    return 0;
}

} // namespace

void AimFinder_Start(void* address, const char* what, bool includeReads)
{
    AimFinder_StartMany(&address, &what, 1, includeReads);
}

void AimFinder_StartMany(void* const* addresses, const char* const* names, int count, bool includeReads)
{
    if (!addresses || count < 1 || !addresses[0] || g_running.exchange(true))
        return;
    if (count > kMaxWatch)
        count = kMaxWatch;

    g_watchCount = 0;
    for (int s = 0; s < count; ++s) {
        if (!addresses[s])
            continue;
        g_watchAddress[g_watchCount] = addresses[s];
        _snprintf_s(g_watchName[g_watchCount], sizeof(g_watchName[0]), _TRUNCATE, "%s",
            (names && names[s]) ? names[s] : "an address");
        ++g_watchCount;
    }
    g_watchReads = includeReads;
    _snprintf_s(g_watchWhat, sizeof(g_watchWhat), _TRUNCATE, "%s",
        (names && names[0]) ? names[0] : "an address");

    if (!g_handler) {
        g_handler = AddVectoredExceptionHandler(1, WatchpointHandler);
        if (!g_handler) {
            Log_Printf("AimFinder: AddVectoredExceptionHandler failed - not arming");
            return;
        }
    }

    HANDLE t = CreateThread(nullptr, 0, &WatchThread, nullptr, 0, nullptr);
    if (t)
        CloseHandle(t);
    else
        Log_Printf("AimFinder: could not start the watch thread");
}

void AimFinder_OnEndScene()
{
    if (!g_done.load(std::memory_order_acquire) || g_reported.exchange(true))
        return;

    const uintptr_t base = reinterpret_cast<uintptr_t>(GetModuleHandleA(nullptr));
    int reported = 0;
    for (const HitEntry& h : g_hits) {
        if (!h.eip)
            continue;
        const uintptr_t eip = static_cast<uintptr_t>(static_cast<DWORD>(h.eip));
        char which[192] = "";
        {
            int w = 0;
            for (int s = 0; s < g_watchCount; ++s) {
                if (!(h.slots & (1L << s)))
                    continue;
                w += sprintf_s(which + w, sizeof(which) - w, "%s[%d] %s", w ? ", " : "", s, g_watchName[s]);
            }
            if (!w)
                sprintf_s(which, sizeof(which), "(which address is unrecorded)");
        }
        Log_Printf("AimFinder: re5dx9.exe+%lX wrote %s - %ld time(s)%s", static_cast<unsigned long>(eip - base),
            which, h.count, h.regsValid ? "" : " (registers missed)");
        if (h.regsValid) {
            Log_Printf("AimFinder:   eax=%08lX ecx=%08lX edx=%08lX ebx=%08lX esi=%08lX edi=%08lX ebp=%08lX esp=%08lX",
                h.eax, h.ecx, h.edx, h.ebx, h.esi, h.edi, h.ebp, h.esp);
        }
        // The code either side (2026-09-22): hooking just after a writer has to
        // land on whole instructions, and the exe is packed on disk, so the
        // running bytes are the only copy there is to read them from. eip is
        // the instruction AFTER the write - the trap fires once it has run.
        {
            unsigned char code[48] = {};
            __try {
                std::memcpy(code, reinterpret_cast<const void*>(eip - 24), sizeof(code));
            } __except (EXCEPTION_EXECUTE_HANDLER) {
            }
            char hex[48 * 3 + 8] = {};
            int w = 0;
            for (int i = 0; i < 48; ++i)
                w += sprintf_s(hex + w, sizeof(hex) - w, i == 24 ? "| %02X " : "%02X ", code[i]);
            Log_Printf("AimFinder:   code -24..+24: %s", hex);
        }
        ++reported;
    }
    if (!reported)
        Log_Printf("AimFinder: nothing wrote it during the window");
    if (g_hitOverflow)
        Log_Printf("AimFinder: %ld more write(s) than the table could hold", g_hitOverflow);
}

bool AimFinder_BusiestHit(AimFinderHit& out)
{
    if (!g_done.load(std::memory_order_acquire))
        return false;
    const HitEntry* best = nullptr;
    for (const HitEntry& h : g_hits) {
        if (!h.eip || !h.regsValid)
            continue;
        if (!best || h.count > best->count)
            best = &h;
    }
    if (!best)
        return false;
    out.eip = static_cast<DWORD>(best->eip);
    out.eax = best->eax;
    out.ecx = best->ecx;
    out.edx = best->edx;
    out.ebx = best->ebx;
    out.esp = best->esp;
    out.ebp = best->ebp;
    out.esi = best->esi;
    out.edi = best->edi;
    out.count = static_cast<DWORD>(best->count);
    return true;
}

void AimFinder_Rearm()
{
    if (g_running.load(std::memory_order_acquire) && !g_done.load(std::memory_order_acquire))
        return; // a window is still open
    for (HitEntry& h : g_hits) {
        InterlockedExchange(&h.eip, 0);
        InterlockedExchange(&h.count, 0);
        InterlockedExchange(&h.regsValid, 0);
    }
    g_hitOverflow = 0;
    g_reported.store(false, std::memory_order_release);
    for (HitEntry& h : g_hits)
        h.slots = 0;
    g_done.store(false, std::memory_order_release);
    g_running.store(false, std::memory_order_release);
}
