#include "log.h"
#include "version.h"

#include <windows.h>
#include <cstdio>
#include <share.h>
#include <cstdarg>
#include <cstring>
#include <mutex>

namespace {

std::mutex g_logMutex;
char g_logPath[MAX_PATH] = {};
FILE* g_logFile = nullptr;

void EnsureLogPath()
{
    if (g_logPath[0] != '\0')
        return;

    HMODULE hSelf = nullptr;
    GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
        reinterpret_cast<LPCSTR>(&EnsureLogPath),
        &hSelf);

    char modulePath[MAX_PATH] = {};
    GetModuleFileNameA(hSelf, modulePath, MAX_PATH);

    char drive[_MAX_DRIVE], dir[_MAX_DIR];
    _splitpath_s(modulePath, drive, sizeof(drive), dir, sizeof(dir), nullptr, 0, nullptr, 0);
    _makepath_s(g_logPath, sizeof(g_logPath), drive, dir, "re5vr", "log");
}

// Opens g_logFile if it isn't already open. Caller must hold g_logMutex.
void EnsureLogFileOpen(const char* mode)
{
    EnsureLogPath();
    if (g_logFile)
        return;
    // _fsopen, not fopen_s (2026-09-19). fopen_s takes the file EXCLUSIVELY -
    // that is the documented difference between the two - so nothing could
    // read re5vr.log while the game was running. Which is precisely when you
    // want to read it: the whole point of the log is to say what is happening
    // now, and being told to quit the game first has cost real time.
    // _SH_DENYWR keeps us the only writer while letting anyone read.
    g_logFile = _fsopen(g_logPath, mode, _SH_DENYWR);
}

// ---- Off the calling thread (2026-09-18) --------------------------------
// Logging used to write and fflush on whichever thread called it. That is a
// syscall, behind a mutex, on the render thread and the XR submit thread.
// Each individual line is cheap enough to ignore; a handful of them on
// three, five and ten second timers are not, because they land on ONE frame
// and a VR frame has an eight millisecond budget. A tester measured a
// rock-solid 120 fps dropping to 96 on a beat, and it survived three
// separate attempts to remove the particular callers responsible.
//
// So the callers stop being the problem. A line is formatted into a fixed
// slot and the caller returns; a writer thread does the file work. Nothing
// is allocated, nothing is freed, and no game thread ever waits on a disk.
//
// The ring drops lines rather than blocking if it is ever outrun - a missing
// diagnostic is worth less than a stalled frame - and it says so when it
// does, so a gap in the log is never silent.
constexpr int kSlots = 512;
constexpr int kSlotChars = 400;

struct Slot {
    volatile LONG ready; // 0 empty, 1 holds a line waiting to be written
    char text[kSlotChars];
};

Slot g_ring[kSlots];
volatile LONG g_writeIndex = 0; // next slot a caller will claim
LONG g_readIndex = 0;           // next slot the writer will drain, writer only
volatile LONG g_dropped = 0;
HANDLE g_writerThread = nullptr;
HANDLE g_wake = nullptr;

void WriteLine(const char* line)
{
    EnsureLogFileOpen("a");
    if (!g_logFile)
        return;
    fputs(line, g_logFile);
    fputc('\n', g_logFile);
}

DWORD WINAPI LogWriter(LPVOID)
{
    for (;;) {
        WaitForSingleObject(g_wake, 250);
        bool wrote = false;
        for (int guard = 0; guard < kSlots * 2; ++guard) {
            Slot& s = g_ring[g_readIndex % kSlots];
            if (!InterlockedCompareExchange(&s.ready, 0, 1))
                break; // nothing waiting in this slot
            {
                std::lock_guard<std::mutex> lock(g_logMutex);
                WriteLine(s.text);
            }
            wrote = true;
            ++g_readIndex;
        }
        const LONG dropped = InterlockedExchange(&g_dropped, 0);
        if (dropped) {
            std::lock_guard<std::mutex> lock(g_logMutex);
            char note[96];
            _snprintf_s(note, sizeof(note), _TRUNCATE, "[log] %ld line(s) dropped - the writer fell behind", dropped);
            WriteLine(note);
            wrote = true;
        }
        if (wrote) {
            std::lock_guard<std::mutex> lock(g_logMutex);
            if (g_logFile)
                fflush(g_logFile);
        }
    }
    return 0;
}

} // namespace

void Log_Init()
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_logFile) {
        fclose(g_logFile);
        g_logFile = nullptr;
    }
    EnsureLogFileOpen("w");
    if (g_logFile) {
        // The build, not just the version (2026-09-23), and taken from the DLL
        // itself rather than from the compiler (2026-09-24).
        //
        // The first attempt used __DATE__ and __TIME__, which bake in when THIS
        // FILE is compiled. This file rarely changes, so its object sat at one
        // timestamp through a dozen builds and every log claimed to be that old
        // build - including logs from machines running the newest one. It was
        // added to end an ambiguity and it created a worse one: twice I read it
        // and told the user a tester was out of date when they were not.
        //
        // The file's own write time cannot be wrong in that way. It is what the
        // person actually copied across, which is the thing being asked about.
        {
            char stamp[64] = "unknown";
            HMODULE self = nullptr;
            if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                        | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                    reinterpret_cast<LPCSTR>(&Log_Printf), &self)
                && self) {
                char path[MAX_PATH] = {};
                if (GetModuleFileNameA(self, path, MAX_PATH)) {
                    WIN32_FILE_ATTRIBUTE_DATA info = {};
                    SYSTEMTIME st = {};
                    FILETIME local = {};
                    if (GetFileAttributesExA(path, GetFileExInfoStandard, &info)
                        && FileTimeToLocalFileTime(&info.ftLastWriteTime, &local)
                        && FileTimeToSystemTime(&local, &st)) {
                        sprintf_s(stamp, "%04u-%02u-%02u %02u:%02u:%02u", st.wYear, st.wMonth, st.wDay, st.wHour,
                            st.wMinute, st.wSecond);
                    }
                }
            }
            fprintf(g_logFile, "RE5VR log started (v" RE5VR_VERSION ", this d3d9.dll dated %s)\n", stamp);
        }

        fflush(g_logFile);
    }
    if (!g_wake)
        g_wake = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    if (!g_writerThread)
        g_writerThread = CreateThread(nullptr, 0, &LogWriter, nullptr, 0, nullptr);
}

void Log_Printf(const char* fmt, ...)
{
    // Format first, on the caller's stack, then hand over a filled slot. The
    // only shared state touched is one interlocked index.
    char line[kSlotChars];
    SYSTEMTIME st;
    GetLocalTime(&st);
    int at = _snprintf_s(line, sizeof(line), _TRUNCATE, "[%02u:%02u:%02u.%03u] ", st.wHour, st.wMinute, st.wSecond,
        st.wMilliseconds);
    if (at < 0)
        at = 0;
    va_list args;
    va_start(args, fmt);
    _vsnprintf_s(line + at, sizeof(line) - at, _TRUNCATE, fmt, args);
    va_end(args);

    if (!g_writerThread) {
        // Before Log_Init, or if the writer could not start: the old way, which
        // is correct if slow, and only happens during load.
        std::lock_guard<std::mutex> lock(g_logMutex);
        WriteLine(line);
        if (g_logFile)
            fflush(g_logFile);
        return;
    }

    const LONG claimed = InterlockedIncrement(&g_writeIndex) - 1;
    Slot& s = g_ring[claimed % kSlots];
    if (s.ready) {
        // The writer has not caught up with this slot yet. Drop rather than
        // wait: a frame is worth more than a line.
        InterlockedIncrement(&g_dropped);
        return;
    }
    std::memcpy(s.text, line, sizeof(s.text));
    InterlockedExchange(&s.ready, 1);
    SetEvent(g_wake);
}

void Log_Flush()
{
    // For the crash handler: drain what is queued and get it on disk before
    // the process goes. Called from a broken process, so it does the work
    // itself rather than waiting for a thread that may never run again.
    //
    // It does NOT stop the writer (2026-09-18). It used to, and the packer
    // throws harmless exceptions while the game loads - so the crash handler
    // ran, flushed, and killed logging for the rest of the session about two
    // seconds in. Every log after that was a startup fragment, which is not a
    // small thing when the log is how everything here gets diagnosed.
    for (int guard = 0; guard < kSlots * 2; ++guard) {
        Slot& s = g_ring[g_readIndex % kSlots];
        if (!InterlockedCompareExchange(&s.ready, 0, 1))
            break;
        WriteLine(s.text);
        ++g_readIndex;
    }
    if (g_logFile)
        fflush(g_logFile);
}
