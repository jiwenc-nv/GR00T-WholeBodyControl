#!/bin/bash
set -e  # Exit on error

# Install TeleopCore wheel (temporary - will move to pypi.nvidia.com)
uv pip install --find-links external_dependencies/isaac_teleop/install/wheels/ teleopcore

export XDG_RUNTIME_DIR=$HOME/.cloudxr/run
export XR_RUNTIME_JSON=$HOME/.cloudxr/share/openxr/1/openxr_cloudxr.json

echo "Dependencies installed successfully. Starting interactive bash shell..."
exec /bin/bash