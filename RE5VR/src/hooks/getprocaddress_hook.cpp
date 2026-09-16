#include "getprocaddress_hook.h"
#include "../proxy/real_d3d9.h"
#include "../util/log.h"

#include <MinHook.h>
#include <windows.h>
#include <intrin.h>

namespace {

typedef FARPROC(WINAPI* GetProcAddress_t)(HMODULE hModule, LPCSTR lpProcName);
GetProcAddress_t oGetProcAddress = nullptr;

// re5dx9.exe itself. Only its own lookups are redirected - see the header.
HMODULE g_hGameModule = nullptr;

bool CallerIsTheGame(void* returnAddress)
{
    if (!g_hGameModule)
        return false;
    HMODULE caller = nullptr;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(returnAddress), &caller)) {
        // No module owns this code: the game's DRM unpacks and runs code from
        // plain allocations, so this is the game too. Only a *different*
        // loaded module is somebody else.
        return true;
    }
    if (caller == g_hGameModule)
        return true;

    static bool logged = false;
    if (!logged) {
        logged = true;
        char name[MAX_PATH] = {};
        GetModuleFileNameA(caller, name, MAX_PATH);
        Log_Printf("hkGetProcAddress: Direct3DCreate9 lookup from %s - left alone (only the game's own lookups "
                   "are redirected); further ones not logged",
            name[0] ? name : "an unknown module");
    }
    return false;
}

bool IsRealString(LPCSTR lpProcName)
{
    // Ordinal-based lookups pass a value with the high bits zero (an
    // integer resource ID, not a pointer) - never safe to treat as a
    // readable string.
    return (reinterpret_cast<ULONG_PTR>(lpProcName) >> 16) != 0;
}

__declspec(noinline) FARPROC WINAPI hkGetProcAddress(HMODULE hModule, LPCSTR lpProcName)
{
    void* const returnAddress = _ReturnAddress();

    FARPROC real = oGetProcAddress(hModule, lpProcName);

    if (IsRealString(lpProcName) && (lstrcmpA(lpProcName, "Direct3DCreate9") == 0
                                        || lstrcmpA(lpProcName, "Direct3DCreate9Ex") == 0)
        && CallerIsTheGame(returnAddress)) {
        if (lstrcmpA(lpProcName, "Direct3DCreate9") == 0) {
            static bool logged = false;
            if (!logged) {
                logged = true;
                Log_Printf("hkGetProcAddress: intercepted Direct3DCreate9 lookup (module=%p) - further lookups not logged", hModule);
            }
            return reinterpret_cast<FARPROC>(&Direct3DCreate9);
        }
        if (lstrcmpA(lpProcName, "Direct3DCreate9Ex") == 0) {
            static bool logged = false;
            if (!logged) {
                logged = true;
                Log_Printf("hkGetProcAddress: intercepted Direct3DCreate9Ex lookup (module=%p) - further lookups not logged", hModule);
            }
            return reinterpret_cast<FARPROC>(&Direct3DCreate9Ex);
        }
    }

    return real;
}

} // namespace

void Hooks_InstallGetProcAddressHook()
{
    g_hGameModule = GetModuleHandleA(nullptr);

    MH_STATUS initSt = MH_Initialize();
    if (initSt != MH_OK && initSt != MH_ERROR_ALREADY_INITIALIZED) {
        Log_Printf("Hooks_InstallGetProcAddressHook: MH_Initialize failed -> %d", static_cast<int>(initSt));
        return;
    }

    HMODULE hKernel32 = GetModuleHandleA("kernel32.dll");
    if (!hKernel32) {
        Log_Printf("Hooks_InstallGetProcAddressHook: GetModuleHandleA(kernel32.dll) failed");
        return;
    }

    void* pTarget = reinterpret_cast<void*>(GetProcAddress(hKernel32, "GetProcAddress"));
    if (!pTarget) {
        Log_Printf("Hooks_InstallGetProcAddressHook: GetProcAddress(GetProcAddress) failed");
        return;
    }

    MH_STATUS st = MH_CreateHook(pTarget, reinterpret_cast<void*>(&hkGetProcAddress),
        reinterpret_cast<void**>(&oGetProcAddress));
    if (st != MH_OK && st != MH_ERROR_ALREADY_CREATED) {
        Log_Printf("Hooks_InstallGetProcAddressHook: MH_CreateHook failed -> %d", static_cast<int>(st));
        return;
    }

    st = MH_EnableHook(pTarget);
    Log_Printf("Hooks_InstallGetProcAddressHook: hook enabled -> %d", static_cast<int>(st));
}
