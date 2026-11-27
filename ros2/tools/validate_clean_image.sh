#!/usr/bin/env bash
# Run on the host. Sources are read-only; build/install live in container /tmp.
set -euo pipefail
workspace="$(cd "$(dirname "$0")/.." && pwd)"
image_name=gbplanner2-humble:local
mkdir -p "$workspace/results/clean_image"
docker image inspect "$image_name" --format '{{json .}}' > "$workspace/results/clean_image/image.json"
docker run --rm --name gbplanner2-clean-validation --network none --platform linux/arm64 \
  --mount "type=bind,source=$workspace,target=/source,readonly" \
  --mount "type=bind,source=$workspace/results/clean_image,target=/evidence" \
  "$image_name" bash -lc '
    mkdir -p /tmp/gbplanner_ws/results
    cp -a /source/src /source/tools /tmp/gbplanner_ws/
    cd /tmp/gbplanner_ws
    rm -f /evidence/exit_status.txt
    bash tools/run_validation.sh > /evidence/validation.log 2>&1
    result=$?
    rm -rf /evidence/results /evidence/unit_tests
    cp -a /tmp/gbplanner_ws/results /evidence/
    mkdir -p /evidence/unit_tests
    cp /tmp/gbplanner_ws/build/gbplanner_graph/test_results/gbplanner_graph/*.gtest.xml \
       /tmp/gbplanner_ws/build/gbplanner_local/test_results/gbplanner_local/*.gtest.xml \
       /tmp/gbplanner_ws/build/gbplanner_voxblox/test_results/gbplanner_voxblox/*.gtest.xml /evidence/unit_tests/
    cp /tmp/gbplanner_ws/build/gbplanner_execution/test_results/gbplanner_execution/*.xunit.xml /evidence/unit_tests/
    dpkg-query -W > /evidence/packages.txt
    printf "%s\n" "$result" > /evidence/exit_status.txt
    tail -30 /evidence/validation.log
    exit "$result"
  '
