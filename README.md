# gr00t_wbc

Software stack for loco-manipulation experiments across multiple humanoid platforms, with primary support for the Unitree G1. This repository provides whole-body control policies, a teleoperation stack, and a data exporter.

---

## System Installation

### Prerequisites
- Ubuntu 22.04
- NVIDIA GPU with a recent driver
- Docker and NVIDIA Container Toolkit (required for GPU access inside the container)

### Repository Setup
Install Git and Git LFS:
```bash
sudo apt update
sudo apt install git git-lfs
git lfs install
```

Clone the repository:
```bash
mkdir -p ~/Projects
cd ~/Projects
git clone --recurse-submodules https://github.com/jiwenc-nv/GR00T-WholeBodyControl.git
cd gr00t_wbc
```

### Docker Environment
We provide a Docker image with all dependencies pre-installed.

Install a fresh image and start a container:
```bash
./docker/run_docker.sh --install --root
```
This pulls the latest `gr00t_wbc` image from `docker.io/nvgear`.

Start or re-enter a container:
```bash
./docker/run_docker.sh --root
```

Use `--root` to run as the `root` user. To run as a normal user, build the image locally:
```bash
./docker/run_docker.sh --build
```
---

## Running the Control Stack

Once inside the container, the control policies can be launched directly.

- Simulation:
  ```bash
  python gr00t_wbc/control/main/teleop/run_g1_control_loop.py
  ```
- Real robot: Ensure the host machine network is configured per the [G1 SDK Development Guide](https://support.unitree.com/home/en/G1_developer) and set a static IP at `192.168.123.222`, subnet mask `255.255.255.0`:
  ```bash
  python gr00t_wbc/control/main/teleop/run_g1_control_loop.py --interface real
  ```

Keyboard shortcuts (terminal window):
- `]`: Activate policy
- `o`: Deactivate policy
- `9`: Release / Hold the robot
- `w` / `s`: Move forward / backward
- `a` / `d`: Strafe left / right
- `q` / `e`: Rotate left / right
- `z`: Zero navigation commands
- `1` / `2`: Raise / lower the base height
- `backspace` (viewer): Reset the robot in the visualizer

---

## Running the Teleoperation Stack

The teleoperation policy primarily uses XR controllers for coordinated hand and body control. It also supports other teleoperation devices, including LeapMotion and HTC Vive with Nintendo Switch Joy-Con controllers.

Keep `run_g1_control_loop.py` running, and in another terminal run:

```bash
python gr00t_wbc/control/main/teleop/run_teleop_policy_loop.py --hand_control_device=isaac --body_control_device=isaac
```

### XR Setup and Controls

Supported hardware:
- [Pico 4 Ultra](https://www.picoxr.com/global/products/pico4-ultra)
- [Meta Quest 3](https://www.meta.com/quest/quest-3)

Prerequisites:
- Early access to https://github.com/NVIDIA/IsaacTeleop
- Connect the XR HMD to the same network as the host computer

Controller bindings:
- `menu + left trigger`: Toggle lower-body policy
- `menu + right trigger`: Toggle upper-body policy
- `Left stick`: X/Y translation
- `Right stick`: Yaw rotation
- `L/R triggers`: Control hand grippers

Isaac Teleop unit test:
```bash
python gr00t_wbc/control/teleop/streamers/isaac_streamer.py
```

---

## Running the Data Collection Stack

Run the full stack (control loop, teleop policy, and camera forwarder) via the deployment helper:
```bash
python scripts/deploy_g1.py \
    --interface sim \
    --camera_host localhost \
    --sim_in_single_process \
    --simulator robocasa \
    --image-publish \
    --enable-offscreen \
    --env_name PnPBottle \
    --hand_control_device=isaac \
    --body_control_device=isaac
```

The `tmux` session `g1_deployment` is created with panes for:
- `control_data_teleop`: Main control loop, data collection, and teleoperation policy
- `camera`: Camera forwarder
- `camera_viewer`: Optional live camera feed

Operations in the `controller` window (`control_data_teleop` pane, left):
- `]`: Activate policy
- `o`: Deactivate policy
- `k`: Reset the simulation and policies
- `` ` ``: Terminate the tmux session
- `ctrl + d`: Exit the shell in the pane

Operations in the `data exporter` window (`control_data_teleop` pane, right top):
- Enter the task prompt

Operations on XR controllers:
- `A`: Start/Stop recording
- `B`: Discard trajectory
