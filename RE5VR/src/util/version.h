#pragma once

// The mod's version, as shown in the menu and compared against the Nexus page.
// Bump this with every release, to the same string typed into the Nexus
// file's version field (e.g. "0.4.1", "0.4.0a").
#define RE5VR_VERSION "0.4.2"

#define RE5VR_WIDEN_(s) L##s
#define RE5VR_WIDEN(s) RE5VR_WIDEN_(s)
#define RE5VR_VERSION_W RE5VR_WIDEN(RE5VR_VERSION)
