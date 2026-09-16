#include "aim_finder.h"

#include "../util/log.h"

#include <windows.h>
#include <tlhelp32.h>

#include <atomic>
#include <cstdio>

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
    DWORD eax, ecx, edx, ebx, esp, ebp, esi, edi; // at first hit
};

HitEntry g_hits[kMaxHits];
volatile LONG g_hitOverflow = 0;
volatile LONG g_recording = 0;
bool g_everArmed = false;

std::atomic<bool> g_running{ false };
std::atomic<bool> g_done{ false };
std::atomic<bool> g_reported{ false };
void* g_watchAddress = nullptr;
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
void ApplyToAllThreads(DWORD address, int* outArmed, int* outFailed)
{
    *outArmed = 0;
    *outFailed = 0;

    DWORD dr7 = 0;
    if (address) {
        dr7 |= 1u;         // L0: local enable for Dr0
        dr7 |= 1u << 16;   // R/W0 = 01: break on write only
        dr7 |= 3u << 18;   // LEN0 = 11: four bytes
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
                c.Dr0 = address;
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
    ApplyToAllThreads(static_cast<DWORD>(reinterpret_cast<uintptr_t>(g_watchAddress)), &armed, &failed);
    Log_Printf("AimFinder: watching writes to %s at %p for %lu ms - armed on %d thread(s), %d failed", g_watchWhat,
        g_watchAddress, kWindowMs, armed, failed);

    Sleep(kWindowMs);

    InterlockedExchange(&g_recording, 0);
    ApplyToAllThreads(0, &armed, &failed);
    Log_Printf("AimFinder: window closed, watchpoints cleared on %d thread(s)", armed);
    g_done.store(true, std::memory_order_release);
    return 0;
}

} // namespace

void AimFinder_Start(void* address, const char* what)
{
    if (!address || g_running.exchange(true))
        return;

    g_watchAddress = address;
    _snprintf_s(g_watchWhat, sizeof(g_watchWhat), _TRUNCATE, "%s", what ? what : "an address");

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
        Log_Printf("AimFinder: re5dx9.exe+%lX wrote it %ld time(s)%s", static_cast<unsigned long>(eip - base),
            h.count, h.regsValid ? "" : " (registers missed)");
        if (h.regsValid) {
            Log_Printf("AimFinder:   eax=%08lX ecx=%08lX edx=%08lX ebx=%08lX esi=%08lX edi=%08lX ebp=%08lX esp=%08lX",
                h.eax, h.ecx, h.edx, h.ebx, h.esi, h.edi, h.ebp, h.esp);
        }
        ++reported;
    }
    if (!reported)
        Log_Printf("AimFinder: nothing wrote it during the window");
    if (g_hitOverflow)
        Log_Printf("AimFinder: %ld more write(s) than the table could hold", g_hitOverflow);
}
