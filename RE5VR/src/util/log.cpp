#include "log.h"

#include <windows.h>
#include <cstdio>
#include <cstdarg>
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
    fopen_s(&g_logFile, g_logPath, mode);
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
        fprintf(g_logFile, "RE5VR log started\n");
        fflush(g_logFile);
    }
}

void Log_Printf(const char* fmt, ...)
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    EnsureLogFileOpen("a");
    if (!g_logFile)
        return;

    SYSTEMTIME st;
    GetLocalTime(&st);
    fprintf(g_logFile, "[%02u:%02u:%02u.%03u] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

    va_list args;
    va_start(args, fmt);
    vfprintf(g_logFile, fmt, args);
    va_end(args);

    fprintf(g_logFile, "\n");
    fflush(g_logFile);
}
