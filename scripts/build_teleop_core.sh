#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Build TeleopCore
#

set -e

# Make sure to run this script from the root of the repository.
GIT_ROOT=$(git rev-parse --show-toplevel)
cd "$GIT_ROOT"

PYTHON_VERSION="3.10"
PYTHON_ABI_CODE=$(echo $PYTHON_VERSION | tr -d .)
PYTHON_ABI_NAME="cp$PYTHON_ABI_CODE-cp$PYTHON_ABI_CODE"

TELEOPCORE_DIR="$GIT_ROOT/external_dependencies/isaac_teleop"
TELEOPCORE_VERSION=$(cat "$TELEOPCORE_DIR/VERSION")

# Build TeleopCore
cd "$TELEOPCORE_DIR"
cmake -B build -DTELEOPCORE_PYTHON_VERSION="$PYTHON_VERSION"
cmake --build build --parallel
cmake --install build