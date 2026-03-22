### Summary

#### From a fresh ROS2 install, need the following dependencies

````sh
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-xacro -y
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y
````

#### Clone submodule for `ur_description` by running this from root directory

````sh
git submodule update --init
````

#### Fix rviz qt segfault issue preventing moveit assistant from working

````sh
wget http://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
sudo dpkg -i ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb 
source install/setup.bash
````

#### Then Run Moveit setup assistant

````sh
ros2 run moveit_setup_assistant moveit_setup_assistant
colcon build
source install/setup.bash
````

#### Then to run moveit demo

````sh
ros2 launch ur5e_isaac_moveit_config controller_v1.launch.py
````

##### To verify if Isaac sim is up and running, and controllers are as well, here are some diagnostic commands that speak for themself

````sh
   71  ros2 topic list
   72  ros2 control list_controllers
   87  ros2 topic echo /robot_description --once
   88  ros2 param get /robot_state_publisher use_sim_time
   89  ros2 param get /controller_manager use_sim_time
   92  ros2 topic info /robot_description --verbose
  107  ros2 topic echo /clock
  108  ros2 node info /controller_manager
  109  ros2 control list_hardware_interfaces
````

#### VLA Controller (OpenPI + ROS2 Bridge)

**Prerequisite:** the `openpi` repo must be cloned into the same root directory as this repo:

````sh
git clone https://github.com/sheilsarda/openpi.git ~/Development/openpi
cd ~/Development/openpi && uv sync
````

The launch file automatically injects `openpi_client` into `PYTHONPATH` — no manual `source .venv/bin/activate` needed at launch time.

> **Note — observation schema for fine-tuned models:** The controller sends raw 6D joint positions to the openpi server. The server-side `Ur5Inputs` transform pads them to 7D, then concatenates the gripper to produce the 8D state the model expects. Do **not** pre-pad joints in the controller — the old DROID baseline did this to match the 7-DOF Franka convention, but applying that padding here would double-pad to 9D and produce garbage inference.

Use two terminals.

##### Terminal 1: start the OpenPI inference server

First, download the fine-tuned checkpoint from HuggingFace if you haven't already (run from the openpi repo):

````sh
cd ~/Development/openpi
python -c "
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id='sheilsarda/pi0_ur5_fast_v1',
    repo_type='model',
    local_dir='checkpoints/pi0_ur5/ur5_fast_v1',
)
"
````

Then serve it:

````sh
cd ~/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi0_ur5 \
  --policy.dir=checkpoints/pi0_ur5/ur5_fast_v1/9999
````

##### Terminal 2: build and launch the ROS2 VLA bridge

**One-time setup** — create a Python 3.12 venv containing only `openpi_client`. This avoids ABI conflicts between the openpi venv (Python 3.11) and the system ROS2 Python (3.12):

````sh
cd ~/Development/VLA_Arm_Controller
python3 -m venv .venv
.venv/bin/pip install openpi-client
````

**Build without the venv active** so the entry point uses system Python 3.12 and can find `cv_bridge`:

````sh
cd ~/Development/VLA_Arm_Controller
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
````

**Dry run first** (infers actions but does not send trajectories — verify action values look sane):

````sh
ros2 launch vla_controller vla_system.launch.py \
  task:="move the arm up" \
  dry_run:=true
````

**Live run** (sends trajectories to the arm):

````sh
ros2 launch vla_controller vla_system.launch.py \
  task:="move the arm up"
````

`vla_system.launch.py` starts the full stack: robot state publisher, controller manager, controller spawners, and the VLA bridge node. Isaac Sim must be running with the ROS2 bridge enabled and the simulation playing before launching.

After starting the VLA system, you can double check controllers are spun up correctly by running the following ros2 command

```sh
sheil@sheil-Precision-7680:~$ ros2 control list_controllers
[INFO] [1774217689.978237450] [_ros2cli_53082]: waiting for service /controller_manager/list_controllers to become available...
ur_manipulator_controller joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster          active
gripper_controller        joint_trajectory_controller/JointTrajectoryController  active
sheil@sheil-Precision-7680:~$ 
```

###### Runtime health logging (stdout)

`vla_controller_node` prints:
- periodic health snapshots
- OpenPI packet receive/fail/invalid counters and latency
- action chunk generated/sent/dropped counters
- action server readiness transitions
- goal success/reject/error counters

Tune verbosity in `src/vla_controller/config/vla_params.yaml`:
- `health_log_period_sec` (`0` disables heartbeat)
- `log_inference_packets`
- `log_action_chunks`

#### Finetuning a VLA

##### Recording Demonstration Episodes

```sh
ros2 bag record /camera/image_raw /camera_wrist/image_raw /joint_states -o ~/Development/VLA_Arm_Controller/training_data/episode_$EPISODE_ID
```

Set `EPISODE_ID` before running, e.g. `EPISODE_ID=001`. Stop with Ctrl+C.

Add this function to `~/.bashrc` for convenience:

```bash
record_episode() {
  ros2 bag record /camera/image_raw /camera_wrist/image_raw /joint_states -o ~/Development/VLA_Arm_Controller/training_data/episode_${1:?usage: record_episode <id>}
}
```

##### Converting Bags to LeRobot Dataset

From the `openpi` repo (no ROS2 environment needed):

```sh
cd /home/sheil/Development/openpi
source .venv/bin/activate
uv run examples/ur5/convert_ur5_bag_to_lerobot.py \
  --bags-dir ~/Development/VLA_Arm_Controller/training_data \
  --repo-id sheilsarda/ur5_isaac_sim_v1 \
  --task "lift the arm up"
```

Output lands in `~/.cache/huggingface/lerobot/sheilsarda/ur5_isaac_sim_v1/`.

##### Inspecting the Dataset with LERO GUI

First-time setup (do this once):

```sh
# Install system OpenCV — needed for AV1 video playback with hardware acceleration.
# The pip version of opencv bundles its own ffmpeg without VA-API support, so it
# can't decode AV1-encoded LeRobot videos. The system package links against the
# system ffmpeg which does.
sudo apt install python3-opencv vainfo nvidia-vaapi-driver

# Verify your GPU supports AV1 hardware decode (look for VAProfileAV1Profile0)
LIBVA_DRIVER_NAME=nvidia vainfo

# Install LERO GUI dependencies (everything except opencv, which comes from system)
cd ~/Development/lero
uv pip install "lero[gui]"
uv pip uninstall opencv-python  # remove the pip version

# Downgrade numpy in the lero venv to match what system OpenCV was compiled against
uv pip install "numpy<2"

# Symlink system cv2 into the lero venv
SYSTEM_CV2=$(/usr/bin/python3 -c "import cv2; print(cv2.__file__)")
ln -s $SYSTEM_CV2 ~/Development/lero/.venv/lib/python3.12/site-packages/
```

Each time you want to inspect a dataset:

```sh
cd ~/Development/lero
source .venv/bin/activate
LIBVA_DRIVER_NAME=nvidia lero ~/.cache/huggingface/lerobot/sheilsarda/ur5_isaac_sim_v1 --gui
```

##### Fine-Tuning (pi0-FAST LoRA)

The model is **pi0-FAST with LoRA** — required to fit within 12GB VRAM. Full pi0 fine-tuning needs ~48GB.

Training is done in Google Colab. See the notebooks in `model_training/`:
- `train_pi0fast_ur5_colab_031026.ipynb` — pi0-FAST (primary)
- `train_pi0_ur5_base_colab_032226.ipynb` — pi0-base (comparison)

Each notebook has three training cells per model: **norm stats**, **from scratch**, and **resume**. Run one of the latter two, not both.

**Current checkpoints on HuggingFace:**
- `sheilsarda/pi0_ur5_fast_v1` — 10k steps, ready for inference
- `sheilsarda/pi0_ur5_base_v1` — not yet trained

**To resume training locally** (if you have a GPU with enough VRAM):

```sh
cd ~/Development/openpi

# From scratch:
uv run scripts/train.py pi0_ur5 --exp-name=ur5_fast_v1 --overwrite --num-train-steps 10000

# Resume from existing checkpoint:
uv run scripts/train.py pi0_ur5 --exp-name=ur5_fast_v1 --resume --num-train-steps 20000
```

- Checkpoints saved to `checkpoints/pi0_ur5/ur5_fast_v1/` every 1,000 steps
- Pretrained weights downloaded automatically from `gs://openpi-assets/checkpoints/pi0_fast_base/params`

> **Note — pi0-base action_dim mismatch:** The pi0-base pretrained checkpoint uses `action_dim=32` (DROID convention). Our UR5 config uses `action_dim=8`. Loading the checkpoint naively causes a shape mismatch on `action_in_proj/kernel` (32×1024 vs 8×1024). The fix is `excluded_prefixes=("action_in_proj", "action_out_proj")` in the `CheckpointWeightLoader` config, which skips those layers and lets them reinitialize randomly for our action space. This is already applied in `openpi/src/openpi/training/config.py` for the `pi0_ur5_base` config.