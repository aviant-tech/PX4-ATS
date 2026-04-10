#!/bin/bash

set -euxo pipefail

CMD="pytest test/ats_tests/test_ats.py $@"

counter=0
while true; do
    counter=$((counter + 1))
    echo $counter
    output=$($CMD | tee /dev/tty)

    if echo $output | grep -q FAILED; then
        exit 0
    fi
done
