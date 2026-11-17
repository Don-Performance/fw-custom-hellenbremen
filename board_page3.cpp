#include "pch.h"
#include "board_page3.h"

board_table_livedata_output_s boardTableLivedataOutput;

// The INI's page = 4 offsets and this struct come from the same generator run; this catches a stale header
static_assert(sizeof(board_table_livedata_output_s) == PAGE_SIZE_3);

uint8_t* boardGetTsPage() {
	return reinterpret_cast<uint8_t*>(&boardTableLivedataOutput);
}
