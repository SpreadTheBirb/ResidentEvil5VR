#pragma once

#include <stddef.h>

// Arm IK across the network (2026-09-19).
//
// RE5's co-op already keeps both characters in step: position, animation, the
// lot. What it cannot know is that one of the players has arms of their own,
// because the arms are ours, solved on our machine out of a pair of controllers
// the game has never heard of. So a VR player looks, to their partner, like
// somebody standing in the stock animation.
//
// The fix needs one thing the game will not give us: the other player's hands.
// This is a side channel for exactly that and nothing else.
//
// What travels, and why it is these numbers rather than the obvious ones:
//
//   * NOT raw controller poses. Those live in each player's own tracking space,
//     which is wherever they happened to recentre. Meaningless at the far end.
//   * The hand's offset FROM THE HEAD, in metres, in the sender's recentred
//     frame - how far right, up and in front of their own head they are holding
//     it. That is a statement about a body, not about a room.
//   * The shoulder's offset from the head, from the sender's T-pose. With it,
//     the receiver can work out hand-relative-to-shoulder, which is the one
//     quantity that means the same thing on two differently sized people.
//   * The controller's axes in that same frame, so the hand turns as well as
//     travels.
//
// The receiver lays all of that onto the PARTNER CHARACTER's own shoulder and
// axes, with its own units-per-metre, exactly the way the local solve does. A
// tall player driving Sheva works, because nothing absolute ever crosses.
//
// Nothing here can change the game's state. It is how the other character is
// DRAWN on your machine and no more, so a lost packet costs a frame of stock
// animation and a dropped link costs nothing at all. That is worth saying
// plainly: this cannot desync a session, because there is no state to desync.
//
// The socket is a plain UDP one of our own on its own port. It is not the
// game's netcode, is not injected into it, and carries nothing but the numbers
// above between two people who are already playing together.

struct IkSyncSettings {
    // Off by default. It opens a socket and talks to another machine, which is
    // not something a mod should ever start doing unasked.
    // ON (2026-09-28, user: "turned on by default so people do not get
    // confused"). It carries hand and shoulder positions only, changes
    // nothing but how the other character is DRAWN on your own machine, and
    // does nothing at all unless your partner is running this build too - so
    // there is no cost to having it on and a real cost to it being hidden.
    bool enabled = true;
    // The partner's address. Only ONE of the two players needs to fill this in:
    // whoever receives a packet learns where it came from and answers there.
    char partnerIp[64] = "";
    // 47045, and deliberately NOT in the 27000s (2026-09-19). The first
    // default was 27045, which is inside Steam's own range of 27000 to 27100,
    // and the bind failed with "port is busy" on the very first attempt to use
    // it. Choosing a port a games platform already lives in was a poor idea
    // for a mod that only ever runs alongside one.
    //
    // Both ends must agree, because a typed address is sent to THIS port. If
    // you change it, change it on both machines.
    int port = 47045;
    // Talk to yourself (2026-09-23, user: "would it be possible to connect to
    // myself in multiplayer to test the IK sync? Idk how that'd be done").
    // Steam will not run the same game twice on one account and sharing will
    // not either, so a second player means a second copy and a second person.
    // But the wire is not the part most likely to be wrong. Everything on
    // either side of it can be tested alone: your own hands are packed into a
    // packet exactly as they would be sent, held back a tenth of a second the
    // way a network would hold them, then fed into the receiving side as
    // though a partner had sent them - and your co-op partner's arms are
    // driven from it.
    //
    // Move your hands and Sheva moves hers. What that proves: the packing,
    // the sanity checks, the merge, the hold when packets stop, the mapping
    // of one person's body onto another character, and the whole partner arm
    // solve. What it cannot prove: that Steam delivers anything.
    bool loopback = false;
};

void IkSync_SetSettings(const IkSyncSettings& s);
IkSyncSettings IkSync_GetSettings();

// Starts or stops the worker thread to match the settings. Safe to call every
// frame; it only acts on a change.
void IkSync_Update();

// Once a frame, from the game's own thread. Does the Steam peer-to-peer half:
// sends this player's hands to the co-op partner and reads back theirs.
// Steam addresses a SteamID rather than an address and a port, so there is
// nothing to type, nothing to forward and no NAT to get through - which is
// exactly where the plain UDP version came unstuck. Cheap and safe to call
// every frame: it returns immediately unless the sync is on and the game has
// told us who the partner is.
void IkSync_Pump();
void IkSync_Shutdown();

// What the partner's hands are doing, in their own body's terms. False when
// nothing has arrived recently.
struct IkSyncHands {
    float handFromHead[2][3];     // metres, in their recentred frame
    float shoulderFromHead[2][3]; // metres, same frame; from their T-pose
    float basis[2][9];            // their controller's axes, same frame
    // How many game units their character's arm measures per real metre of
    // THEIR arm, from their own T-pose. It has to travel, because only the
    // sender knows how long their real arm is - the receiver can measure the
    // character (it is the same character on both machines) but not the
    // person driving it. 0 when they have not calibrated.
    float unitsPerMetre[2];
    bool haveHand[2];
    bool haveShoulder[2];
    // Both of their hands are on their gun, so put the partner's off hand on
    // the gun here too rather than leaving it in mid air.
    bool twoHanded;
    unsigned long long ms;
};
bool IkSync_GetPartnerHands(IkSyncHands& out);

// One line for the menu: who we are talking to and whether anything is coming
// back.
// The Steam route: whether it is available at all, who the game says the
// partner is (0 until it has spoken to them), and the traffic in the last full
// second. This is the one that matters now - the address and port below it are
// the fallback for when Steam is not there.
bool IkSync_GetSteamStatus(unsigned long long* partner, unsigned* sentPerSecond, unsigned* gotPerSecond);

void IkSync_DescribeStatus(char* out, size_t size);
