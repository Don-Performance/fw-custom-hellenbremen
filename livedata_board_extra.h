// Board hook included by rusEFI's console/binary/live_data.cpp. The board directory comes
// first on the include path, so this file replaces the empty core placeholder.
// Declares the board live data structs listed in board_live_data.yaml. Their headers are
// generated into generated/live_data_generated/, which rusEFI does not put on the include path.
#pragma once

#include "generated/live_data_generated/board_autotune_state_generated.h"
#include "generated/live_data_generated/board_at_idle_base_state_generated.h"
#include "generated/live_data_generated/board_actuator_state_generated.h"
