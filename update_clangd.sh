#!/bin/bash
set -e

# ============================================================
#  Script: update_clangd.sh
#  Muc dich: Tao file compile_commands.json hoan chinh cho clangd,
#  bao gom day du duong dan ARM GCC, Mbed OS va MJLib.
# ============================================================

# Buoc 1: Tao file compile_commands.json goc
echo "=== Buoc 1: Tao file compile_commands.json goc ==="
rm -f compile_commands.json
bazel run //:refresh_compile_commands -- --config=target

# Buoc 2: Thu thap duong dan include
echo "=== Buoc 2: Thu thap duong dan include ==="

# Thu muc Mbed OS (platform)
PLATFORM_DIRS=$(find /work/external/com_github_ARMmbed_mbed-g4/platform/ -type d | sed 's/^/-isystem /')

# Thu vien ARM GCC
ARM_GCC_INCLUDES=(
    "-isystem" "/usr/lib/gcc/arm-none-eabi/9.2.1/../../../arm-none-eabi/include/c++/9.2.1"
    "-isystem" "/usr/lib/gcc/arm-none-eabi/9.2.1/../../../arm-none-eabi/include/c++/9.2.1/arm-none-eabi"
    "-isystem" "/usr/lib/gcc/arm-none-eabi/9.2.1/../../../arm-none-eabi/include/c++/9.2.1/backward"
    "-isystem" "/usr/lib/gcc/arm-none-eabi/9.2.1/include"
    "-isystem" "/usr/lib/gcc/arm-none-eabi/9.2.1/include-fixed"
    "-isystem" "/usr/lib/gcc/arm-none-eabi/9.2.1/../../../arm-none-eabi/include"
)

# Thu vien MJLib
MJLIB_INCLUDES=(
    "-isystem" "/work/external/com_github_mjbots_mjlib/mjlib"
    "-isystem" "/work/external/com_github_mjbots_mjlib"
)

# Buoc 3: Cap nhat noi dung file JSON
echo "=== Buoc 3: Cap nhat noi dung compile_commands.json ==="
jq --argjson gcc_includes "$(printf '%s\n' "${ARM_GCC_INCLUDES[@]}" | jq -R . | jq -s .)" \
   --argjson mjlib_includes "$(printf '%s\n' "${MJLIB_INCLUDES[@]}" | jq -R . | jq -s .)" \
   --arg platform_includes "$PLATFORM_DIRS" '
map(
    (.arguments[0] |= "/usr/bin/arm-none-eabi-g++") |
    (.arguments += $gcc_includes) |
    (.arguments += ($platform_includes | split("\n") | map(select(length > 0)))) |
    (.arguments += $mjlib_includes) |
    (.arguments += ["-D_SYS__TIMEVAL_H_"])
)
' compile_commands.json > compile_commands.json.NEW

# Buoc 4: Hoan tat
echo "=== Buoc 4: Hoan tat ==="
mv compile_commands.json.NEW compile_commands.json
echo "File compile_commands.json da duoc cap nhat thanh cong."
echo "Hay 'Reload Window' hoac 'Restart clangd' trong VS Code de ap dung thay doi."