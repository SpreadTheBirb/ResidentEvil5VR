#include "pose_guard.h"

#include "../util/log.h"

#include <windows.h>
#include <tlhelp32.h>

#include <atomic>
#include <cstring>

// See pose_guard.h for why this exists. The debug-register mechanics follow
// aim_finder.cpp, which does the same arming across every thread; the
// differences are that this one stays armed, watches four addresses instead of
// one, and writes from the handler rather than only recording.

namespace {

constexpr int kSlots = 4;  // debug registers, and there are only four
constexpr int kHeld = 16;  // joints whose pose we put back, which can be more

struct Slot {
    volatile LONG active;   // 0 = unused
    unsigned char* address; // the watched bytes: the rotation's last component
};

// What to restore. Deliberately longer than the watch list (2026-09-18): the
// wrists need holding too and six joints will not fit in four registers. They
// do not need to - the blend writes the whole arm in one pass, so a trap on any
// watched joint means the others have just been written as well. Restoring
// every held joint on every trap covers all of them for four registers' worth
// of watching, at the price of a few extra memcpys per trap.
struct Held {
    volatile LONG valid;
    int index;
    unsigned char* joints; // whose skeleton, so two characters cannot collide
    unsigned char* rotation;
    float quat[4];
    // The joint's first eight bytes (its class word and its id/parent links)
    // when the hold was taken. A swapped skeleton frees this memory and the
    // game reuses it; writing a quaternion into whatever lives there now is
    // heap corruption, so nothing is put back unless these still match.
    unsigned int sig[2];
};

Slot g_slots[kSlots];
Held g_held[kHeld];
PVOID g_handler = nullptr;
std::atomic<bool> g_armed{ false };
std::atomic<unsigned long> g_saves{ 0 };
// One skeleton per slot: "whose is this" travels with the joint rather than
// being a property of the guard as a whole.
unsigned char* g_slotJoints[kSlots] = {};
int g_stride = 0, g_rotOffset = 0;
int g_indices[kSlots] = { -1, -1, -1, -1 };
int g_count = 0;

// The handler writes to an address it is watching, which traps again. One
// level of that is expected and harmless as long as the second pass does
// nothing; deeper would mean something else is wrong, so it bails.
__declspec(thread) int t_inHandler = 0;

LONG CALLBACK GuardHandler(EXCEPTION_POINTERS* ep)
{
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP)
        return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT* ctx = ep->ContextRecord;
    const DWORD fired = static_cast<DWORD>(ctx->Dr6) & 0xF;
    if (!fired) {
        // Ours only if we are armed; a stray trace step is not.
        if (!g_armed.load(std::memory_order_relaxed))
            return EXCEPTION_CONTINUE_SEARCH;
        ctx->Dr6 = 0;
        return EXCEPTION_CONTINUE_EXECUTION;
    }
    if (t_inHandler) {
        // Our own write. Swallow it and carry on.
        ctx->Dr6 = 0;
        return EXCEPTION_CONTINUE_EXECUTION;
    }
    ++t_inHandler;
    for (int i = 0; i < kHeld; ++i) {
        Held& h = g_held[i];
        if (!h.valid || !h.rotation)
            continue;
        __try {
            const unsigned int* base = reinterpret_cast<const unsigned int*>(h.rotation - g_rotOffset);
            // Id (byte 3) and parent (byte 1) only (2026-09-22). Comparing all
            // eight bytes caught state flags changing as the gun came up, took
            // that for a swapped skeleton and dropped the hold - which handed
            // the arm to the animation whenever the player aimed.
            if ((base[1] & 0xFF00FF00u) != (h.sig[1] & 0xFF00FF00u)) {
                InterlockedExchange(&h.valid, 0);
                continue;
            }
            std::memcpy(h.rotation, h.quat, sizeof(h.quat));
            g_saves.fetch_add(1, std::memory_order_relaxed);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            // The character went away underneath us; the next Set fixes it.
        }
    }
    --t_inHandler;
    ctx->Dr6 = 0;
    return EXCEPTION_CONTINUE_EXECUTION;
}

// Threads already carrying the watchpoints, so a sweep can skip them.
// Suspending seventy threads to rewrite registers they already hold is what a
// tester saw as a rhythmic stutter - a clean 120 to 100 and back, once per
// sweep, for work that changed nothing (2026-09-18).
constexpr int kKnownThreads = 256;
DWORD g_knownThreads[kKnownThreads] = {};
int g_knownCount = 0;

bool AlreadyArmed(DWORD id)
{
    for (int i = 0; i < g_knownCount; ++i) {
        if (g_knownThreads[i] == id)
            return true;
    }
    return false;
}

// Arms (or clears, when every address is null) the four watchpoints. onlyNew
// skips threads that already have them, which is every thread on a sweep where
// the game has not made any. Nothing between Suspend and Resume may take a lock.
void ApplyToAllThreads(int* outArmed, int* outFailed, bool onlyNew)
{
    DWORD dr7 = 0;
    DWORD addr[kSlots] = {};
    for (int i = 0; i < kSlots; ++i) {
        if (!g_slots[i].active || !g_slots[i].address)
            continue;
        addr[i] = static_cast<DWORD>(reinterpret_cast<uintptr_t>(g_slots[i].address));
        dr7 |= 1u << (i * 2);              // Ln: local enable
        dr7 |= 1u << (16 + i * 4);         // R/Wn = 01: on write
        dr7 |= 3u << (18 + i * 4);         // LENn = 11: four bytes
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
        if (onlyNew && AlreadyArmed(te.th32ThreadID))
            continue;
        HANDLE t = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT | THREAD_SUSPEND_RESUME, FALSE,
            te.th32ThreadID);
        if (!t) {
            if (outFailed)
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
                c.Dr7 = dr7;
                armed = SetThreadContext(t, &c) != FALSE;
            }
            ResumeThread(t);
        }
        if (armed) {
            if (outArmed)
                ++*outArmed;
            if (g_knownCount < kKnownThreads)
                g_knownThreads[g_knownCount++] = te.th32ThreadID;
        } else if (outFailed) {
            ++*outFailed;
        }
        CloseHandle(t);
    }
    CloseHandle(snap);
}

std::atomic<bool> g_sweepOnlyNew{ false };

DWORD WINAPI ArmThread(LPVOID)
{
    int armed = 0, failed = 0;
    const bool onlyNew = g_sweepOnlyNew.exchange(false, std::memory_order_acq_rel);
    ApplyToAllThreads(&armed, &failed, onlyNew);
    if (armed || !onlyNew)
        Log_Printf("PoseGuard: holding the arm pose on %d more thread(s), %d failed", armed, failed);
    return 0;
}

void ReArm()
{
    HANDLE t = CreateThread(nullptr, 0, &ArmThread, nullptr, 0, nullptr);
    if (t)
        CloseHandle(t);
}

} // namespace

void PoseGuard_Set(const PoseGuardSlot* slots, int count, int jointStride, int rotOffset)
{
    if (!slots || count <= 0 || jointStride <= 0)
        return;
    if (count > kSlots)
        count = kSlots;

    bool same = count == g_count && jointStride == g_stride && rotOffset == g_rotOffset;
    for (int i = 0; i < count && same; ++i) {
        if (g_indices[i] != slots[i].index || g_slotJoints[i] != slots[i].joints)
            same = false;
    }
    if (same && g_armed.load(std::memory_order_relaxed))
        return;

    g_stride = jointStride;
    g_rotOffset = rotOffset;
    g_count = count;
    for (int i = 0; i < kSlots; ++i) {
        const bool use = i < count && slots[i].joints && slots[i].index >= 0;
        g_indices[i] = use ? slots[i].index : -1;
        g_slotJoints[i] = use ? slots[i].joints : nullptr;
        Slot& s = g_slots[i];
        if (use) {
            // The last component: by the time it is written the whole rotation
            // has been set, so putting ours back cannot leave a mongrel of the
            // two - which is what watching the FIRST component did, and it made
            // quaternions of length 1.09.
            s.address = slots[i].joints + slots[i].index * jointStride + rotOffset + 12;
            InterlockedExchange(&s.active, 1);
        } else {
            s.address = nullptr;
            InterlockedExchange(&s.active, 0);
        }
    }
    for (int i = 0; i < kHeld; ++i) {
        InterlockedExchange(&g_held[i].valid, 0);
        g_held[i].index = -1;
        g_held[i].joints = nullptr;
        g_held[i].rotation = nullptr;
    }

    if (!g_handler) {
        g_handler = AddVectoredExceptionHandler(1, GuardHandler);
        if (!g_handler) {
            Log_Printf("PoseGuard: AddVectoredExceptionHandler failed - the pose cannot be held");
            return;
        }
    }
    g_armed.store(true, std::memory_order_release);
    g_knownCount = 0; // the addresses changed, so every thread needs the new ones
    ReArm();
}

void PoseGuard_Hold(unsigned char* joints, int index, const float quat[4])
{
    if (!quat || !joints || index < 0)
        return;
    int free = -1;
    for (int i = 0; i < kHeld; ++i) {
        if (g_held[i].valid && g_held[i].index == index && g_held[i].joints == joints) {
            std::memcpy(g_held[i].quat, quat, sizeof(g_held[i].quat));
            return;
        }
        if (!g_held[i].valid && free < 0)
            free = i;
    }
    if (free < 0)
        return;
    g_held[free].index = index;
    g_held[free].joints = joints;
    g_held[free].rotation = joints + index * g_stride + g_rotOffset;
    __try {
        const unsigned int* base = reinterpret_cast<const unsigned int*>(joints + index * g_stride);
        g_held[free].sig[0] = base[0];
        g_held[free].sig[1] = base[1];
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    std::memcpy(g_held[free].quat, quat, sizeof(g_held[free].quat));
    InterlockedExchange(&g_held[free].valid, 1);
}

void PoseGuard_Clear()
{
    if (!g_armed.exchange(false, std::memory_order_acq_rel))
        return;
    for (int i = 0; i < kSlots; ++i) {
        InterlockedExchange(&g_slots[i].active, 0);
        g_slots[i].address = nullptr;
    }
    for (int i = 0; i < kHeld; ++i) {
        InterlockedExchange(&g_held[i].valid, 0);
        g_held[i].rotation = nullptr;
        g_held[i].index = -1;
    }
    for (int i = 0; i < kSlots; ++i)
        g_slotJoints[i] = nullptr;
    g_count = 0;
    g_knownCount = 0;
    ReArm(); // with every slot inactive this clears the registers
    Log_Printf("PoseGuard: released the arm pose");
}

void PoseGuard_Tick()
{
    if (!g_armed.load(std::memory_order_relaxed))
        return;
    // A watchpoint lives on the threads it was set on, and the game makes more
    // as it goes. Re-arming every couple of seconds catches them without
    // suspending everything every frame.
    static ULONGLONG s_lastMs = 0;
    const ULONGLONG now = GetTickCount64();
    if (now - s_lastMs < 3000)
        return;
    s_lastMs = now;
    g_sweepOnlyNew.store(true, std::memory_order_release);
    ReArm();
}

unsigned long PoseGuard_TakeSaveCount()
{
    return g_saves.exchange(0, std::memory_order_relaxed);
}
