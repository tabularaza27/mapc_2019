#!/bin/bash

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cp -av "$SCRIPT_DIRECTORY/../group5_server_configs/." "$SCRIPT_DIRECTORY/../../../third-party/massim/server/conf"


