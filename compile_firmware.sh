#!/usr/bin/env bash
export EXTRA_LIVE_DATA_FILE=",../../../board_live_data.yaml"
cd ext/rusefi/firmware/
bash bin/compile.sh ../../../meta-info.env
