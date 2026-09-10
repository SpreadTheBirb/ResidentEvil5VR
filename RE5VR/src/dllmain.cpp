#include <windows.h>
#include "util/log.h"
#include "proxy/real_d3d9.h"
#include "hooks/getprocaddress_hook.h"

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    switch (reason) {
    case DLL_PROCESS_ATTACH:
        DisableThreadLibraryCalls(hModule);
        Log_Init();
        Log_Printf("RE5VR proxy DLL attached (module=%p)", hModule);

        // re5dx9.exe statically imports at least one d3d9.dll export
        // (D3DPERF_GetStatus) that the loader/game-startup code calls
        // before ever touching Direct3DCreate9. Our blind-forward stubs
        // jump through function pointers that must be resolved before that
        // first call, so resolve the real d3d9.dll here rather than lazily.
        RealD3D9_Init();

        // The game reaches its Direct3D9-rendered main menu without ever
        // calling this proxy's own Direct3DCreate9 export - it resolves the
        // function dynamically against what appears to be a separately,
        // explicitly-loaded genuine d3d9.dll module, bypassing ordinary
        // DLL search order entirely. Hooking GetProcAddress itself catches
        // that lookup regardless of which module it's resolved against.
        Hooks_InstallGetProcAddressHook();
        break;
    case DLL_PROCESS_DETACH:
        Log_Printf("RE5VR proxy DLL detaching");
        break;
    }
    return TRUE;
}
