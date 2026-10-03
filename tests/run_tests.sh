#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/encos-tests.XXXXXX")
trap 'rm -rf "$test_build_dir"' EXIT
compiler=${CC:-cc}
flags=(-std=c11 -Wall -Wextra -Werror -pedantic -g)
if [[ ${SANITIZE:-0} == 1 ]]; then
    flags+=(-fsanitize=address,undefined -fno-omit-frame-pointer)
fi
"$compiler" "${flags[@]}" -I. encos_protocol.c tests/test_encos_protocol.c -lm -o "$test_build_dir/protocol"
"$test_build_dir/protocol"
"$compiler" "${flags[@]}" -Itests/stubs -I. encos_protocol.c MT4_motor.c \
    controller.c pid.c monitor.c tests/test_mt4_driver.c -lm -o "$test_build_dir/driver"
"$test_build_dir/driver"
"$compiler" "${flags[@]}" -D_HAL_MT4MOTOR_ENABLE=0 -Itests/stubs -I. \
    -c MT4_motor.c -o "$test_build_dir/disabled.o"
