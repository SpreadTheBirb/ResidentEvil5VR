#pragma once

// The magazine, and how to reach it (2026-09-25).
//
// Manual reloading needs three numbers - how many rounds are in the gun, how
// many the gun can hold, and how many are left in the bag - and a way to put
// new ones in. All three sit together on the heap, and a week of scanning
// never produced a pointer path to them that survived a relaunch: the
// structure is allocated wherever it happens to land, and nothing in the
// exe's own data points at it in any way a scan could follow.
//
// So the address is not looked for at all. A hardware watchpoint on the live
// magazine, with a single shot fired, named the instruction that writes it:
//
//     re5dx9.exe+5EDD05   89 56 1C    mov [esi+1Ch], edx
//     re5dx9.exe+5EDD08   E8 ...      call ...
//
// esi is the magazine, edx is the new count, and it was loaded two
// instructions earlier by "mov edx,[edi+8]" - so the count is being copied
// INTO the weapon FROM somewhere else, and that somewhere is what a reload
// has to change if it is to stick. ebx held the record it came from.
//
// A five byte jump at the call - a whole instruction, which the write is not,
// and one we know runs because the trap fired on it - hands us esi and ebx
// every time the game touches ammunition. No path, no scan, no address that
// goes stale between launches. The same trick as the pose and camera hooks.
//
// Offsets from esi, measured against the counts the finder confirmed:
//
//     +1Ch   rounds in the gun
//     +2Ch   what the gun can hold, upgrades included
//     +34h   rounds left for it
//
// Reads only, until Ammo_SetLoaded is called. In process, always: writing
// game memory from outside blue-screened the machine once already.

struct AmmoSlot {
    unsigned char* mag = nullptr;   // esi: the weapon's own ammunition
    unsigned char* owner = nullptr; // edi: the record the count is copied FROM, at +8
    unsigned long hits = 0;         // how often the game has written it
    int loaded = 0, capacity = 0, reserve = 0;
    unsigned mark = 0;              // esi+24h, 0201h on every weapon, so not an id
    unsigned id = 0;                // esi+14h, which read 1 on the M92F - the weapon
    unsigned state = 0;             // esi+18h
    unsigned long long lastMs = 0;  // when it last changed
    bool alive = false;             // and whether it still reads back
};

// Places the jump. Says in the log whether the instruction was where it should
// be. Safe to call every frame; it only ever runs once.
void Ammo_Install();
bool Ammo_Installed();

// Refreshes what each known magazine holds. Once a frame, from EndScene.
void Ammo_OnEndScene();

// Every magazine the hook has seen this session, busiest first.
int Ammo_Slots(AmmoSlot* out, int max);

// The one the game is working on: the most recently written of them, which
// while you are shooting is the gun in your hand.
bool Ammo_Held(AmmoSlot& out);

// Both structures, word by word, into the log. The way to see what else lives
// near the count - the owner record's layout is still unknown.
void Ammo_Dump();

// Put rounds in, or take them out. Writes the weapon's copy and, when the
// owner record is known, the copy it is refreshed from - otherwise the next
// sync would simply put the old number back.
bool Ammo_SetLoaded(int rounds);
bool Ammo_SetReserve(int rounds);

// A reload, in the two halves the gesture has. Ejecting empties the gun;
// seating fills it with as much as the bag can give, which is
// min(capacity, reserve), and takes exactly that from the bag.
// WHERE THE SPARE ROUNDS REALLY LIVE (2026-09-25, user: "it doesn't lower my
// current ammo capacity though ... I stayed at my full inventory capacity of
// ammo, never changed").
//
// The count at +34h is a mirror. Writing it works for a second and then the
// game puts the old number straight back, exactly as the rounds in the gun
// would have done if the record behind them were not written too - so the
// spare rounds have a record behind them as well, and it has not been found.
// This looks for it three ways in one press: the number, wherever it sits in
// either structure; the same number one pointer out from the record; and a
// watchpoint on the mirror, so whatever refreshes it names itself.
void Ammo_FindTheBag();

bool Ammo_Eject(bool keepTheRounds);
bool Ammo_Seat(int* tookOut);
