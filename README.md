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

Use two terminals.

##### Terminal 1: start the OpenPI inference server from the `openpi` repo:

For the fine-tuned UR5 model (after training — see Fine-Tuning section below):

````sh
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi0_ur5 \
  --policy.dir=checkpoints/pi0_ur5/ur5_lift_v1/5000
````

For zero-shot DROID baseline (before fine-tuning):

````sh
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py --env=DROID
````

##### Terminal 2: build and launch the full ROS2 side (controller manager + spawners + VLA bridge, no `move_group`):

````sh
cd /home/sheil/Development/VLA_Arm_Controller
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vla_controller
source install/setup.bash

ros2 launch vla_controller vla_system.launch.py \
  task:="pick up the block" \
  openpi_host:=localhost \
  openpi_port:=8000
````

**Appendix: If controllers are already running in another terminal/session, launch only the bridge node:**

````sh
ros2 launch vla_controller vla.launch.py \
  task:="pick up the block" \
  openpi_host:=localhost \
  openpi_port:=8000
````

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

**Step 1 — Compute norm stats** (run once, or after changing dataset/config):

```sh
cd ~/Development/openpi
uv run scripts/compute_norm_stats.py --config-name=pi0_ur5
```

**Step 2 — Train:**

```sh
XLA_PYTHON_CLIENT_MEM_FRACTION=0.85 uv run scripts/train.py pi0_ur5 \
  --exp-name=ur5_lift_v1 --overwrite
```

- Checkpoints saved to `checkpoints/pi0_ur5/ur5_lift_v1/` every 1,000 steps
- Pretrained weights downloaded automatically from `gs://openpi-assets/checkpoints/pi0_fast_base/params`
- `--overwrite` is safe — only deletes the local experiment dir, not the pretrained GCS weights