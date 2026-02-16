#!/bin/bash
set -e  # Exit on error

# Install TeleopCore wheel (TeleopCore at Isaac workspace: ../../TeleopCore)
uv pip install --find-links ../../TeleopCore/install/wheels/ teleopcore

export XDG_RUNTIME_DIR=$HOME/.cloudxr/run
export XR_RUNTIME_JSON=$HOME/.cloudxr/share/openxr/1/openxr_cloudxr.json

echo "Dependencies installed successfully. Starting interactive bash shell..."
exec /bin/bash