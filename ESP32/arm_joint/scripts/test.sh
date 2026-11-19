#!/bin/sh
# 作用：独立编译真实控制核心并执行内存/未定义行为检查；用法：从任意目录执行本脚本。
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
out=$(mktemp -d /tmp/arm-joint-test.XXXXXX)
trap 'rm -rf "$out"' EXIT
clang -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I"$root/include" "$root/src/arm_core.c" "$root/test/test_core.c" -lm -o "$out/test_core"
"$out/test_core"
