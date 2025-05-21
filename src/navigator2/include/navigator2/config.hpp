#pragma once

#include <cstdint>

namespace map_cfg {

static constexpr double COORDINATE_TOLERANCE = 1;

enum class OptimalPathMode : std::uint8_t {
    MIN_DISTANCE = 0,
    MIN_TIME = 1
};

} // namespace map_cfg