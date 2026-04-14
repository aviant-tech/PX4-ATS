#!/bin/bash

set -euxo pipefail

counter=0
while true; do
    counter=$((counter + 1))
    echo $counter
    output=$(pytest test/ats_tests/test_ats.py "$@" | tee /dev/tty)

    if echo $output | grep -q FAILED; then
        exit 0
    fi
done
