#!/bin/bash

declare -a CMAKE_PLATFORM_FLAGS

cmake -G Ninja \
 -DCMAKE_BUILD_TYPE=Release \
 -DCMAKE_INSTALL_PREFIX=$PREFIX \
  ${CMAKE_PLATFORM_FLAGS[@]} \
 -DCMAKE_PREFIX_PATH=$PREFIX \
 -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
 \
 -DUSE_BUILD_SCRIPT_OUTPUT=Off \
 -DUSE_STATIC_MSVC_RUNTIME=Off \
 -DBoost_USE_STATIC_LIBS=Off \
 -DIFC_INCLUDE_DIR=$PREFIX/include \
 -DIFC_LIBRARY_DIR=$PREFIX/lib \
 -DOCC_INCLUDE_DIR=$PREFIX/include/opencascade \
 -DOCC_LIBRARY_DIR=$PREFIX/lib \
 .

ninja

ninja install