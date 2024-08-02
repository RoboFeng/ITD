#pragma once

#include <unistd.h>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N_STD 10000000000
#define MAX_FRAME_N 20000

// Collect all keypoints of keyframes using voxel maps
class VOXEL_LOC
{
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// Hash value
template <>
struct std::hash<VOXEL_LOC>
{
    int64_t operator()(const VOXEL_LOC &s) const
    {
        using std::hash;
        using std::size_t;
        return ((((s.z) * HASH_P) % MAX_N_STD + (s.y)) * HASH_P) % MAX_N_STD + (s.x);
    }
};

class STDesc_LOC
{
public:
    int64_t x, y, z;

    STDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const STDesc_LOC &other) const
    {
        // use three attributes
        return (x == other.x && y == other.y && z == other.z);
    }
};

template <>
struct std::hash<STDesc_LOC>
{
    int64_t operator()(const STDesc_LOC &s) const
    {
        using std::hash;
        using std::size_t;
        return ((((s.z) * HASH_P) % MAX_N_STD + (s.y)) * HASH_P) % MAX_N_STD + (s.x);
    }
};