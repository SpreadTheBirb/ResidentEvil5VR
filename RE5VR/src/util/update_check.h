#pragma once

#include <string>

// Asks Nexus Mods for the mod page's current version, once, on a background
// thread. Nexus' public GraphQL API answers this without an API key, so no
// key ships in the mod and nothing about the player is sent.

enum class UpdateState {
    Off,       // the player turned the check off
    Checking,
    UpToDate,
    Available, // Nexus has a newer version
    Failed,    // no answer, or one we couldn't read
};

struct UpdateStatus {
    UpdateState state = UpdateState::Off;
    std::string latest; // Nexus' version string, when we got one
};

// Starts a check unless one is already running. Safe to call from any thread.
void UpdateCheck_Start();
void UpdateCheck_SetOff();
UpdateStatus UpdateCheck_GetStatus();

// The mod's Nexus page, for an "open in browser" button.
const char* UpdateCheck_NexusUrl();
void UpdateCheck_OpenNexusPage();

// Compares version strings like "0.4.1" and "0.4.0a": numbers part by part,
// then any trailing letters. Negative, zero or positive like strcmp.
int UpdateCheck_CompareVersions(const char* a, const char* b);
