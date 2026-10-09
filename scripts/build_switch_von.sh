#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "== Building Virtual-On for Nintendo Switch =="

if [ -z "$DEVKITPRO" ]; then
    echo "DEVKITPRO not set, running via Docker (devkitpro-mesa-rust:latest)..."
    docker run --rm \
        -v "${ROOT_DIR}/..":/work \
        -w "/work/$(basename "${ROOT_DIR}")" \
        devkitpro-mesa-rust:latest \
        bash -c "export PATH=/opt/devkitpro/devkitA64/bin:/opt/devkitpro/tools/bin:\$PATH && \
                 cmake -S platform/switch -B build-switch-von -DCMAKE_TOOLCHAIN_FILE=/opt/devkitpro/cmake/Switch.cmake -DCMAKE_BUILD_TYPE=Release -DM2_ROMSET=von && \
                 cmake --build build-switch-von -j\$(nproc)"
else
    export PATH="${DEVKITPRO}/devkitA64/bin:${DEVKITPRO}/tools/bin:${PATH}"
    cmake -S "${ROOT_DIR}/platform/switch" -B "${ROOT_DIR}/build-switch-von" \
        -DCMAKE_TOOLCHAIN_FILE="${DEVKITPRO}/cmake/Switch.cmake" \
        -DCMAKE_BUILD_TYPE=Release \
        -DM2_ROMSET=von
    cmake --build "${ROOT_DIR}/build-switch-von" -j$(nproc)
fi

cp "${ROOT_DIR}/build-switch-von/von_switch.nro" "${ROOT_DIR}/../von.nro"
echo "== Successfully built: von.nro =="
