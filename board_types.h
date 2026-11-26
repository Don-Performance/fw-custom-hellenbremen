#pragma once

#include <cstdint>

enum class ccfcModes_e : uint8_t {
    On   = 0,
    Auto = 1,
    Off  = 2,
};

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
// "255.255.255" plus terminating NUL
using dpecm_openblt_version_t = char[12];
// Fixed length, not null-terminated, like dpecm_hardware_id_t
using dpecm_manufacturer_t = char[4];
// "YYYY-MM-DD HH:MM:SS" plus terminating NUL
using dpecm_date_t = char[20];
