#pragma once

#include <cmath>

// Minimal row-major 3x3 matrix helpers shared by the head-tracking code:
// openvr_bridge.cpp extracts an HMD rotation delta from OpenVR poses, and
// stereo_test.cpp composes it with RE5's own camera basis (decoded from
// the c0-c3 vertex shader constants - see stereo_test.cpp for the
// decomposition this supports).
//
// Row-major, row-vector convention throughout: a matrix's row i is the
// i-th basis vector (Right/Up/Forward) expressed in its own "world" space,
// and Mat3ApplyRowVector(m, v, out) computes out = v * m.

struct Mat3 {
    float m[9]; // row-major: m[row*3+col]
};

inline Mat3 Mat3Identity()
{
    Mat3 r{};
    r.m[0] = 1.0f;
    r.m[4] = 1.0f;
    r.m[8] = 1.0f;
    return r;
}

inline Mat3 Mat3Transpose(const Mat3& a)
{
    Mat3 r{};
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            r.m[col * 3 + row] = a.m[row * 3 + col];
    return r;
}

inline Mat3 Mat3Multiply(const Mat3& a, const Mat3& b)
{
    Mat3 r{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a.m[row * 3 + k] * b.m[k * 3 + col];
            r.m[row * 3 + col] = sum;
        }
    }
    return r;
}

// Converts an OpenXR/glTF-style unit quaternion (x, y, z, w) into this
// project's row-major Right/Up/Forward Mat3 convention - new for the
// OpenXR migration (Phase 5): OpenVR's HMD pose came pre-shaped as a 3x4
// matrix (see openvr_bridge.cpp's old VRSubmitThreadProc, which just read
// columns directly), so this project never needed quaternion math before.
// Standard column-major quaternion-to-rotation-matrix derivation (columns
// = where local X/Y/Z map to in the parent space), transposed into rows
// here since local X/Y IS Right/Up directly, and local Z is the OpenXR
// convention's "backward" axis, so Forward is column 2 negated - the same
// "columns 0/1/2 are Right/Up/-Forward" pattern already used for OpenVR's
// HMD pose and OpenXR's own eye-to-head transform elsewhere in this
// project, just derived from a quaternion instead of a matrix.
inline Mat3 QuaternionToMat3(float x, float y, float z, float w)
{
    Mat3 r{};
    r.m[0] = 1.0f - 2.0f * (y * y + z * z); r.m[1] = 2.0f * (x * y + z * w);        r.m[2] = 2.0f * (x * z - y * w);
    r.m[3] = 2.0f * (x * y - z * w);        r.m[4] = 1.0f - 2.0f * (x * x + z * z); r.m[5] = 2.0f * (y * z + x * w);
    r.m[6] = -2.0f * (x * z + y * w);       r.m[7] = -2.0f * (y * z - x * w);       r.m[8] = -1.0f + 2.0f * (x * x + y * y);
    return r;
}

// Builds a local-frame rotation delta (rows = new Right/Up/Forward
// expressed as combinations of the OLD Right/Up/Forward - same convention
// as the head-tracking delta in openvr_bridge.cpp, suitable for the same
// Mat3Multiply(delta, gameBasis)-style composition, e.g. via
// stereo_test.cpp's ApplyHeadRotation) that rotates a basis's Right/
// Forward axes around its own Up axis, leaving Up unchanged. A plain,
// hand-verified rotation matrix - unlike a hardware-sourced transform
// (e.g. OpenVR's per-eye toe-in, tried and reverted in this project after
// producing a badly broken render - see re5vr project notes), this is
// simple enough to trust without external validation. Positive
// angleRadians rotates Forward toward -Right (intuitively, "turn left").
inline Mat3 Mat3RotateAroundUp(float angleRadians)
{
    Mat3 r = Mat3Identity();
    const float c = std::cos(angleRadians);
    const float s = std::sin(angleRadians);
    r.m[0] = c;    r.m[1] = 0.0f; r.m[2] = s;
    r.m[3] = 0.0f; r.m[4] = 1.0f; r.m[5] = 0.0f;
    r.m[6] = -s;   r.m[7] = 0.0f; r.m[8] = c;
    return r;
}
