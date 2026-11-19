#!/bin/sh
# 作用：分别验证主轴独立运行和从板看门狗，检查内存/未定义行为；用法：从任意目录执行，依赖clang。
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
out=$(mktemp -d /tmp/arm-joint-test.XXXXXX)
trap 'rm -rf "$out"' EXIT
for role in 1 0; do
  clang -std=c11 -D_POSIX_C_SOURCE=200809L -DARM_MASTER="$role" \
    -Wall -Wextra -Werror -fsanitize=address,undefined \
    -I"$root/include" "$root/src/arm_core.c" "$root/test/test_core.c" -lm -o "$out/test_core_$role"
  "$out/test_core_$role"
done
