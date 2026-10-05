#!/usr/bin/env bash
# Build and run the host-side tests. Needs only a C++17 compiler -- no board,
# no Arduino toolchain, no AVR cross-compiler.
set -euo pipefail
cd "$(dirname "$0")"
CXX="${CXX:-c++}"
FLAGS=(-std=c++17 -I. -Istubs -Wall -Wextra -Wno-unused-parameter -O1)
status=0
for suite in test_regressions test_behaviour; do
  echo "building $suite"
  "$CXX" "${FLAGS[@]}" -o "/tmp/arm_$suite" "$suite.cpp"
  "/tmp/arm_$suite" || status=1
  echo
done
if [ "$status" -eq 0 ]; then echo "all suites passed"; else echo "FAILURES -- see above"; fi
exit "$status"
