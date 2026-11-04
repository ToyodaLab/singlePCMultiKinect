#pragma once
// PacketV2.h
// CommonGround "CGS2" sensor packet (version 2). Header-only and free of SDK types so it can be unit tested.
//
// Compared with the original 580-byte packet it adds:
//   * bodyId      - the body tracker's persistent id for this person on this sensor, so the receiver can follow one
//                   person from frame to frame without guessing from positions;
//   * timestampUs - the capture time on the device clock, so frames from different sensors can be ordered by when
//                   they were captured instead of when they happened to arrive;
//   * float32 positions in metres instead of half-float millimetres (about 0.3 mm instead of ~1 cm at 3 m).
// Joint orientations are not sent: the receiver derives orientations from the fused positions.
//
// Layout (all little endian), 437 bytes:
//   0   char[4]  "CGS2"
//   4   uint8    version = 2
//   5   uint8    kind    = 1 (sensor frame)
//   6   uint16   sensorId   (index of the device in the calibration file for the room)
//   8   uint32   bodyId     (0xFFFFFFFF if unknown)
//   12  uint64   timestampUs
//   20  uint8    joint count = 32
//   21  32 x { float32 x, y, z  (metres, Kinect camera space: +X right, +Y down, +Z forward), uint8 confidence 0..3 }

#include <cstdint>
#include <cstring>
#include <vector>

namespace cg {

constexpr size_t kPacketV2JointCount = 32;
constexpr size_t kPacketV2Size = 21 + kPacketV2JointCount * 13;
constexpr uint32_t kUnknownBodyId = 0xFFFFFFFFu;

inline void putLE16(std::vector<uint8_t>& out, uint16_t v)
{
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

inline void putLE32(std::vector<uint8_t>& out, uint32_t v)
{
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}

inline void putLE64(std::vector<uint8_t>& out, uint64_t v)
{
    for (int i = 0; i < 8; ++i) out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}

inline void putFloatLE(std::vector<uint8_t>& out, float f)
{
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    putLE32(out, bits);
}

// positionsMm: 32 x (x, y, z) in millimetres as reported by the body tracker.
// confidence:  32 values 0..3 (k4abt_joint_confidence_level_t).
inline std::vector<uint8_t> buildSensorPacketV2(uint16_t sensorId, uint32_t bodyId, uint64_t timestampUs,
                                                const float* positionsMm, const uint8_t* confidence)
{
    std::vector<uint8_t> p;
    p.reserve(kPacketV2Size);

    p.push_back('C'); p.push_back('G'); p.push_back('S'); p.push_back('2');
    p.push_back(2);  // version
    p.push_back(1);  // kind: sensor frame
    putLE16(p, sensorId);
    putLE32(p, bodyId);
    putLE64(p, timestampUs);
    p.push_back(static_cast<uint8_t>(kPacketV2JointCount));

    for (size_t j = 0; j < kPacketV2JointCount; ++j) {
        putFloatLE(p, positionsMm[j * 3 + 0] * 0.001f);
        putFloatLE(p, positionsMm[j * 3 + 1] * 0.001f);
        putFloatLE(p, positionsMm[j * 3 + 2] * 0.001f);
        p.push_back(confidence[j] > 3 ? 3 : confidence[j]);
    }
    return p;
}

} // namespace cg
