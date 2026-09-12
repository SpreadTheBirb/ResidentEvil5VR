#include "crash_log.h"
#include "log.h"

#include <windows.h>

#include <cstdio>
#include <cstring>

namespace {

// A vectored handler sees FIRST-chance exceptions, including the many
// harmless ones a game and its runtimes throw and handle internally (C++
// throws, the 0x406D1388 thread-naming ping, guard-page hits). Logging those
// would bury the real crash, so only genuinely fatal codes are reported, and
// only a few times.
bool IsFatalCode(DWORD code)
{
    switch (code) {
    case EXCEPTION_ACCESS_VIOLATION:
    case EXCEPTION_ILLEGAL_INSTRUCTION:
    case EXCEPTION_PRIV_INSTRUCTION:
    case EXCEPTION_STACK_OVERFLOW:
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
    case EXCEPTION_DATATYPE_MISALIGNMENT:
    case EXCEPTION_IN_PAGE_ERROR:
    case EXCEPTION_INT_DIVIDE_BY_ZERO:
    case EXCEPTION_NONCONTINUABLE_EXCEPTION:
        return true;
    default:
        return false;
    }
}

// Name an address the way a debugger would: "module.dll+1A2B3". Uses the
// GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT form so this never alters a
// module's lifetime while the process is already falling over.
void DescribeAddress(void* address, char* out, size_t outSize)
{
    HMODULE module = nullptr;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            static_cast<LPCSTR>(address), &module)
        && module) {
        char path[MAX_PATH] = {};
        GetModuleFileNameA(module, path, MAX_PATH);
        const char* name = std::strrchr(path, '\\');
        name = name ? name + 1 : path;
        const uintptr_t offset = reinterpret_cast<uintptr_t>(address) - reinterpret_cast<uintptr_t>(module);
        _snprintf_s(out, outSize, _TRUNCATE, "%s+%IX", name, offset);
    } else {
        _snprintf_s(out, outSize, _TRUNCATE, "%p (no module - freed code? bad function pointer?)",
            address);
    }
}

// A flat cap was a mistake: Steam's DRM throws three handled exceptions at
// startup (2026-09-12), which used up a budget of 3 and silently swallowed
// the real crash minutes later. Dedupe per site instead, so a repeating
// known-harmless fault can never crowd out a new one.
constexpr int kMaxSites = 24;
constexpr LONG kMaxPerSite = 2;
struct Site {
    volatile LONG code;
    volatile LONG address;
    volatile LONG count;
};
Site g_sites[kMaxSites];

// True if this exact (code, address) still has budget left.
bool ShouldReport(DWORD code, void* address)
{
    const LONG key = static_cast<LONG>(reinterpret_cast<uintptr_t>(address));
    const LONG codeKey = static_cast<LONG>(code);
    for (int i = 0; i < kMaxSites; ++i) {
        if (g_sites[i].address == key && g_sites[i].code == codeKey)
            return InterlockedIncrement(&g_sites[i].count) <= kMaxPerSite;
        if (g_sites[i].address == 0 &&
            InterlockedCompareExchange(&g_sites[i].address, key, 0) == 0) {
            g_sites[i].code = codeKey;
            return InterlockedIncrement(&g_sites[i].count) <= kMaxPerSite;
        }
    }
    return false; // table full - stop rather than spam
}

LONG CALLBACK OnException(EXCEPTION_POINTERS* info)
{
    if (!info || !info->ExceptionRecord)
        return EXCEPTION_CONTINUE_SEARCH;

    const EXCEPTION_RECORD* rec = info->ExceptionRecord;
    if (!IsFatalCode(rec->ExceptionCode))
        return EXCEPTION_CONTINUE_SEARCH;
    if (!ShouldReport(rec->ExceptionCode, rec->ExceptionAddress))
        return EXCEPTION_CONTINUE_SEARCH;

    char where[MAX_PATH + 64] = {};
    DescribeAddress(rec->ExceptionAddress, where, sizeof(where));

    Log_Printf("*** CRASH: code 0x%08lX at %s (thread %lu)", rec->ExceptionCode, where, GetCurrentThreadId());

    if (rec->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && rec->NumberParameters >= 2) {
        const ULONG_PTR kind = rec->ExceptionInformation[0];
        Log_Printf("*** CRASH: %s address %p",
            kind == 0 ? "reading" : (kind == 1 ? "writing" : "executing"),
            reinterpret_cast<void*>(rec->ExceptionInformation[1]));
    }

    // Return addresses off the stack, named the same way. Not a real stack
    // walk - no symbols here - but enough to see which DLLs are on the way
    // in, which is the whole question when a runtime call disappears.
    if (info->ContextRecord) {
        const CONTEXT* ctx = info->ContextRecord;
        Log_Printf("*** CRASH: eip=%p esp=%p ebp=%p", reinterpret_cast<void*>(ctx->Eip),
            reinterpret_cast<void*>(ctx->Esp), reinterpret_cast<void*>(ctx->Ebp));

        const void** sp = reinterpret_cast<const void**>(ctx->Esp);
        int printed = 0;
        for (int i = 0; i < 256 && printed < 8; ++i) {
            const void* candidate = nullptr;
            if (IsBadReadPtr(sp + i, sizeof(void*)))
                break;
            candidate = sp[i];
            // Only values that look like code in a loaded module.
            MEMORY_BASIC_INFORMATION mbi = {};
            if (!VirtualQuery(candidate, &mbi, sizeof(mbi)) || mbi.State != MEM_COMMIT)
                continue;
            const DWORD exec = PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
            if (!(mbi.Protect & exec) || mbi.Type != MEM_IMAGE)
                continue;
            char frame[MAX_PATH + 64] = {};
            DescribeAddress(const_cast<void*>(candidate), frame, sizeof(frame));
            Log_Printf("*** CRASH:   stack[%d] %s", i, frame);
            ++printed;
        }
    }

    // Log_Printf fflushes every line, so the report is already on disk.
    return EXCEPTION_CONTINUE_SEARCH; // let the game/OS handle it as usual
}

} // namespace

void CrashLog_Install()
{
    // First=1: run before the game's own handlers, so a crash the game
    // swallows is still recorded.
    if (AddVectoredExceptionHandler(1, &OnException))
        Log_Printf("CrashLog: crash reporter installed");
    else
        Log_Printf("CrashLog: AddVectoredExceptionHandler failed (%lu)", GetLastError());
}
