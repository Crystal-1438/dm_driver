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
