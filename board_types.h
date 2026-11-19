#pragma once

#include <cstdint>

enum class autotuneCellChangeResistance_e : uint8_t {
    VeryHigh = 0,
    High     = 1,
    Normal   = 2,
    Low      = 3,
    VeryLow  = 4,
};

// C++ side of the custom string type in board_config.txt (like vin_number_t in rusefi_types.h).
// The size must match the literal there.
using dpecm_hardware_id_t = char[24];
using dpecm_hardware_rev_t = char[16];
