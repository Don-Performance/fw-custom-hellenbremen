#pragma once

#include "pch.h"
#include "page_3_generated.h"

// TS page 4 (TS_PAGE_BOARD), defined in board_page3.txt: autotune results, RAM-only and read-only for TS
extern board_table_livedata_output_s boardTableLivedataOutput;

uint8_t* boardGetTsPage();
