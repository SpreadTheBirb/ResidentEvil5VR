#include "bone_palette.h"

#include "../hooks/arm_ik.h"
#include "../util/log.h"

#include <windows.h>

#include <cmath>
#include <cstring>
#include <limits>

// WHERE, NOT WHICH (2026-09-26).
//
// Five attempts went into learning WHICH bone each entry of the shader's table
// was, and that is genuinely hard: the table is a per-draw subset of up to 32
// bones in an order the renderer picks, and naming a bone needs millimetre
// precision in a skeleton where fingers sit beside each other.
//
// Hiding never needed that. It needs to know WHERE an entry is, and every
// entry carries its own world position in the fourth column of its matrix. Ask
// which of your joints it is sitting on and a centimetre of error changes no
// answer. The keep set itself - which joints survive Full body, Arms or Hands -
// is worked out in arm_ik.cpp from the skeleton's own parent links, where the
// hierarchy is, and published for this file to read.
//
// AND THIS IS WHY IT IS DONE HERE (2026-09-26, user: "my screen actually went
// completely black and my fps tanked").
//
// The previous build hid the body by writing NaN into the skeleton's world
// matrices. That works on the mesh and is a disaster everywhere else: eighteen
// places in this mod read those matrices, and the first one to read a hidden
// joint carried NaN out into the eye position, from there into the culling
// frustum, and from there into every visibility test in the game. The log has
// it plainly - "the game took the camera - it was -nan(ind) units from where
// the body puts the eye" - with the frame rate falling from sixteen to five
// behind it.
//
// This table is different in the one way that matters: it is on its way to the
// GPU and nothing reads it back. NaN put in here cannot reach the eye, the
// solve or the next frame. A vertex weighted to a NaN bone is NaN, the hardware
// discards any triangle holding one, and it is gone without being anywhere.

namespace {

constexpr unsigned kTableBase = 23; // measured: every large upload starts here
constexpr int kRegsPerBone = 3;     // and every size is a multiple of twelve
constexpr int kMaxSlots = 85;       // 256 registers is the cap upstream

// About 85 game units to the metre. Thirteen centimetres is comfortably wider
// than the gap between neighbouring joints and far narrower than the gap to
// anybody else standing near you, which is what keeps a co-op partner's limbs
// out of this entirely.
constexpr float kMineRadius = 0.13f * 85.0f;

unsigned long g_blanked = 0, g_kept = 0, g_theirs = 0;
// HOW FAR OUT THE MISSES ARE (2026-09-26, user: "partly hidden" - chunks of
// the body stay drawn and stay put rather than flickering).
//
// Stable chunks mean a whole batch is being read as somebody else's, not that
// the keep set is wrong. The question is by how much it misses: entries a few
// centimetres outside the radius mean the radius is simply too tight, and
// entries hundreds of units out mean that batch's matrices are not in world
// space at all and matching them by position can never work.
unsigned long g_band[5] = {}; // <11, 11-30, 30-100, 100-1000, beyond

// WHAT IS IN AN ENTRY (2026-09-26, user: "turning on the modes just seems to
// hide the right hand, nothing else").
//
// The bands say the rest of the body misses by tens to hundreds of units, and
// the one thing that DOES hide sits where the weapon is parented. That is the
// signature of these matrices not being world matrices at all but skinning
// matrices - inverse bind pose times world - whose translation is only near
// the joint for bones whose bind pose happens to sit near the model origin.
//
// If that is what they are, matching by position cannot be made to work by
// widening anything, and this prints enough to settle it: a few entries beside
// the joints they ought to be landing on.
unsigned long long g_dumpedAt = 0;
unsigned long long g_toldAt = 0;
bool g_had = false;

} // namespace

void BonePalette_SetMode(int, bool)
{
    // The mode lives in the settings and the keep set is built from it in
    // arm_ik.cpp. Nothing to do here; kept so the old call sites still link.
}

bool BonePalette_Filter(unsigned startRegister, const float* data, unsigned count, float* out)
{
    if (startRegister != kTableBase || !data || !out)
        return false;
    if (count < kRegsPerBone || count % kRegsPerBone != 0)
        return false;
    const int slots = static_cast<int>(count) / kRegsPerBone;
    if (slots > kMaxSlots)
        return false;

    ArmIkBodyHide body;
    g_had = ArmIk_BodyHide(body);
    if (!g_had)
        return false;

    const unsigned long long nowMs = GetTickCount64();
    const bool dumping = slots >= 8 && nowMs - g_dumpedAt > 5000;
    if (dumping) {
        g_dumpedAt = nowMs;
        Log_Printf("BonePalette: a table of %d bone(s). Your joints run %.0f %.0f %.0f to %.0f %.0f %.0f", slots,
            body.pos[0][0], body.pos[0][1], body.pos[0][2], body.pos[body.count - 1][0],
            body.pos[body.count - 1][1], body.pos[body.count - 1][2]);
        for (int k = 0; k < slots && k < 6; ++k) {
            const float* e = data + k * 12;
            Log_Printf("BonePalette:   entry %d translation %.1f %.1f %.1f | row0 %.3f %.3f %.3f", k, e[3], e[7],
                e[11], e[0], e[1], e[2]);
        }
    }

    bool copied = false;
    for (int k = 0; k < slots; ++k) {
        const float* e = data + k * 12;
        // A 4x3 matrix, row major: the translation is the last element of each
        // row rather than a fourth row.
        const float p[3] = { e[3], e[7], e[11] };

        int nearest = -1;
        float bestD = kMineRadius * kMineRadius;
        float nearKept = 1e30f; // and how close the nearest SURVIVING joint is
        for (int j = 0; j < body.count; ++j) {
            if (!body.ok[j])
                continue;
            const float dx = p[0] - body.pos[j][0];
            const float dy = p[1] - body.pos[j][1];
            const float dz = p[2] - body.pos[j][2];
            const float d = dx * dx + dy * dy + dz * dz;
            if (d < bestD) {
                bestD = d;
                nearest = j;
            }
            if (body.keep[j] && d < nearKept)
                nearKept = d;
        }
        // Not near any joint of yours, so it is not yours: a partner, a
        // creature, a prop. Never ours to edit.
        if (nearest < 0) {
            ++g_theirs;
            // How near it came, so a miss can be told from a stranger.
            float outBy = 1e30f;
            for (int j = 0; j < body.count; ++j) {
                if (!body.ok[j])
                    continue;
                const float dx = p[0] - body.pos[j][0];
                const float dy = p[1] - body.pos[j][1];
                const float dz = p[2] - body.pos[j][2];
                const float d = dx * dx + dy * dy + dz * dz;
                if (d < outBy)
                    outBy = d;
            }
            outBy = std::sqrt(outBy);
            ++g_band[outBy < 30.0f ? 1 : outBy < 100.0f ? 2 : outBy < 1000.0f ? 3 : 4];
            continue;
        }
        ++g_band[0];
        // Kept outright, or near enough to something kept that cutting it
        // would take the cuff off the glove. The second test is what the cut
        // slider moves, and it is smooth because it is a distance rather than
        // a choice of joint.
        if (body.keep[nearest] || nearKept < body.cutUnits * body.cutUnits) {
            ++g_kept;
            continue;
        }
        if (!copied) {
            std::memcpy(out, data, sizeof(float) * count * 4);
            copied = true;
        }
        // NOTHING CALLS THIS ANY MORE (2026-09-28). The matching above
        // cannot work: these are skinning matrices and their translations
        // are nowhere near the joints they belong to - measured at nearly
        // three thousand units apart in a live run. Kept for the log it
        // produces and as a record of a route that was tried and closed.
        float* o = out + k * 12;
        o[0] = 0.0f; o[1] = 0.0f; o[2] = 0.0f; o[3] = p[0];
        o[4] = 0.0f; o[5] = 0.0f; o[6] = 0.0f; o[7] = p[1];
        o[8] = 0.0f; o[9] = 0.0f; o[10] = 0.0f; o[11] = p[2];
        ++g_blanked;
    }
    return copied;
}

void BonePalette_OnEndScene()
{
    const unsigned long long now = GetTickCount64();
    if (!g_blanked && !g_kept && !g_theirs)
        return;
    if (now - g_toldAt < 5000)
        return;
    g_toldAt = now;
    Log_Printf("BonePalette: last 5 s - %lu entries hidden, %lu kept on you, %lu somebody else's%s", g_blanked,
        g_kept, g_theirs, g_had ? "" : " (nothing published to hide)");
    Log_Printf("BonePalette: how near the unmatched came - %lu within reach, %lu under 30 units, %lu under 100, "
               "%lu under 1000, %lu further",
        g_band[0], g_band[1], g_band[2], g_band[3], g_band[4]);
    g_blanked = g_kept = g_theirs = 0;
    for (int i = 0; i < 5; ++i)
        g_band[i] = 0;
}

void BonePalette_Relearn()
{
}

bool BonePalette_Status(int& blanked, int& kept, int& theirs)
{
    blanked = static_cast<int>(g_blanked);
    kept = static_cast<int>(g_kept);
    theirs = static_cast<int>(g_theirs);
    return g_had;
}
