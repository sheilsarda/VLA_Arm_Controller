# Plan: Integrating IsaacSim-ros_workspaces Jazzy Workspace

## Executive Summary

This document outlines a plan to integrate the Jazzy workspace from the `IsaacSim-ros_workspaces` repository into the current project. The integration will provide access to Isaac Sim ROS2 packages, MoveIt integration, navigation packages, and other robotics components needed for Isaac Sim development.

---

## Current State Analysis

### IsaacSim-ros_workspaces Repository Structure

The repository contains:
- **`jazzy_ws/`** - ROS2 Jazzy workspace (target for integration)
- **`humble_ws/`** - ROS2 Humble workspace (not needed for this project)
- **Git submodules** - Required dependencies:
  - `jazzy_ws/src/moveit/moveit_resources` (branch: `isaac_moveit_tutorial_jazzy`)
  - `jazzy_ws/src/moveit/topic_based_ros2_control`
- **Build scripts** - Docker-based build system (`build_ros.sh`)
- **Configuration files** - `fastdds.xml` for FastDDS configuration

### Jazzy Workspace Contents

The `jazzy_ws/src/` directory contains the following packages:

1. **`ackermann_control/`** - Ackermann steering control
2. **`custom_message/`** - Custom ROS2 message definitions
3. **`humanoid_locomotion_policy_example/`** - Humanoid locomotion examples
4. **`isaac_ros2_messages/`** - Isaac Sim ROS2 message types
5. **`isaac_tutorials/`** - Isaac Sim tutorial packages
6. **`isaacsim/`** - Core Isaac Sim ROS2 integration package
7. **`moveit/`** - MoveIt integration packages (with submodules)
8. **`navigation/`** - Navigation stack packages

### Current Project State

- **DevContainer**: Configured for ROS2 Jazzy with user `sheil_u24`
- **Workspace**: `/home/ws` with `src/` directory structure
- **Post-create command**: Already configured for `rosdep` installation

---

## Integration Options

### Option 1: Git Submodule (Recommended)

**Approach:**
- Add `IsaacSim-ros_workspaces` as a git submodule
- Use sparse checkout to get only `jazzy_ws/` directory
- Initialize required submodules recursively

**Pros:**
- Easy to update from upstream
- Clear dependency tracking
- Maintains link to original repository
- Minimal storage (only needed files)

**Cons:**
- Requires submodule management
- Team members need to understand submodules
- Slightly more complex initial setup

**Implementation:**
```bash
# Add as submodule
git submodule add <isaacsim-ros-workspaces-url> IsaacSim-ros_workspaces

# Configure sparse checkout for jazzy_ws only
cd IsaacSim-ros_workspaces
git config core.sparseCheckout true
echo "jazzy_ws/*" > .git/info/sparse-checkout
echo "build_ros.sh" >> .git/info/sparse-checkout
echo ".gitmodules" >> .git/info/sparse-checkout
git read-tree -mu HEAD

# Initialize submodules
git submodule update --init --recursive
```

### Option 2: Copy Selected Packages

**Approach:**
- Copy only needed packages from `jazzy_ws/src/` into project `src/`
- Manually handle submodule dependencies
- Copy `fastdds.xml` if needed

**Pros:**
- Full control over included packages
- No submodule complexity
- Can customize packages as needed
- Simpler for team members unfamiliar with submodules

**Cons:**
- Manual updates required
- Potential code duplication
- Lose connection to upstream updates
- Must manually track submodule dependencies

**Implementation:**
```bash
# Copy specific packages
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaacsim src/
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaac_ros2_messages src/
# ... copy other needed packages

# Handle submodules manually
git clone --branch isaac_moveit_tutorial_jazzy \
  https://github.com/ayushgnv/moveit_resources.git \
  src/moveit/moveit_resources
```

### Option 3: Sparse Checkout with Selective Package Copy

**Approach:**
- Use sparse checkout to get `jazzy_ws/`
- Copy selected packages into main workspace `src/`
- Keep submodule references

**Pros:**
- Balance between control and maintainability
- Can selectively update packages
- Maintains some connection to upstream

**Cons:**
- More complex setup
- Requires understanding of both approaches

---

## Recommended Approach: Option 1 (Git Submodule)

### Rationale

1. **Maintainability**: Easy to pull updates from NVIDIA's repository
2. **Dependency Management**: Automatic handling of submodules
3. **Storage Efficiency**: Only downloads what's needed
4. **Industry Standard**: Common practice for ROS workspace dependencies

---

## Detailed Integration Plan

### Phase 1: Repository Setup

#### 1.1 Add IsaacSim-ros_workspaces as Submodule

```bash
# From project root
git submodule add https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces.git IsaacSim-ros_workspaces
git submodule update --init --recursive
```

#### 1.2 Configure Sparse Checkout

```bash
cd IsaacSim-ros_workspaces
git config core.sparseCheckout true

# Create sparse-checkout file
cat > .git/info/sparse-checkout << EOF
jazzy_ws/*
build_ros.sh
.gitmodules
dockerfiles/ubuntu_22_jazzy_python_312_minimal.dockerfile
dockerfiles/ubuntu_24_jazzy_python_312_minimal.dockerfile
EOF

# Apply sparse checkout
git read-tree -mu HEAD
cd ..
```

#### 1.3 Initialize Required Submodules

The `.gitmodules` file specifies:
- `jazzy_ws/src/moveit/moveit_resources` (required if using MoveIt)
- `jazzy_ws/src/moveit/topic_based_ros2_control` (required if using MoveIt)

```bash
cd IsaacSim-ros_workspaces
git submodule update --init --recursive jazzy_ws/src/moveit/moveit_resources
git submodule update --init --recursive jazzy_ws/src/moveit/topic_based_ros2_control
cd ..
```

### Phase 2: Workspace Integration

#### 2.1 Determine Package Requirements

**Critical packages (likely needed):**
- `isaacsim/` - Core Isaac Sim ROS2 integration
- `isaac_ros2_messages/` - Message definitions
- `isaac_tutorials/` - Example code and tutorials

**Optional packages (depending on use case):**
- `moveit/` - If using MoveIt for motion planning
- `navigation/` - If using navigation stack
- `ackermann_control/` - If using wheeled robots
- `humanoid_locomotion_policy_example/` - If working with humanoids
- `custom_message/` - If using custom messages

#### 2.2 Integration Strategy

**Option A: Symlink Approach**
```bash
# Create symlinks in main workspace
ln -s ../IsaacSim-ros_workspaces/jazzy_ws/src/isaacsim src/isaacsim
ln -s ../IsaacSim-ros_workspaces/jazzy_ws/src/isaac_ros2_messages src/isaac_ros2_messages
# ... other packages
```

**Option B: Copy to Workspace**
```bash
# Copy packages into main workspace
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaacsim src/
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaac_ros2_messages src/
# ... other packages
```

**Option C: Use Entire jazzy_ws as Workspace**
```bash
# Use jazzy_ws directly as the workspace
# Update devcontainer to point to jazzy_ws
# Add custom packages alongside Isaac packages
```

#### 2.3 Recommended: Hybrid Approach

- **Core Isaac packages**: Symlink from submodule (easy updates)
- **Custom packages**: Copy or develop in main `src/`
- **MoveIt packages**: Symlink if using MoveIt, otherwise exclude

### Phase 3: Configuration Updates

#### 3.1 Update DevContainer Configuration

**Current `.devcontainer/devcontainer.json`:**
- Already configured for ROS2 Jazzy
- `postCreateCommand` runs `rosdep install` from `src/`

**Potential updates:**
```json
{
  "postCreateCommand": "sudo rosdep update && sudo rosdep install --from-paths src --ignore-src -y && sudo chown -R $(whoami) /home/ws/ && cd IsaacSim-ros_workspaces && git submodule update --init --recursive && if [ -d ~/.ssh ]; then chmod 700 ~/.ssh && chmod 600 ~/.ssh/* 2>/dev/null || true && chmod 644 ~/.ssh/*.pub 2>/dev/null || true; fi"
}
```

#### 3.2 FastDDS Configuration

If using FastDDS (instead of default DDS):
```bash
# Copy fastdds.xml to workspace root or appropriate location
cp IsaacSim-ros_workspaces/jazzy_ws/fastdds.xml .
# Or symlink
ln -s IsaacSim-ros_workspaces/jazzy_ws/fastdds.xml fastdds.xml
```

#### 3.3 Environment Variables

May need to set:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # If using FastDDS
# Or
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # If using CycloneDDS
```

### Phase 4: Build System Integration

#### 4.1 Colcon Build Configuration

The workspace should build with standard colcon:
```bash
cd /home/ws
colcon build --symlink-install
source install/setup.bash
```

#### 4.2 Dependency Resolution

Ensure all dependencies are available:
```bash
# Update rosdep
rosdep update

# Install dependencies for all packages
rosdep install --from-paths src --ignore-src -y
```

#### 4.3 Handle Submodule Dependencies

MoveIt submodules may have their own dependencies:
```bash
cd IsaacSim-ros_workspaces/jazzy_ws/src/moveit
rosdep install --from-paths . --ignore-src -y
```

---

## Package Selection Guide

### Essential Packages (Start Here)

1. **`isaacsim/`**
   - **Purpose**: Core package for launching Isaac Sim as ROS2 node
   - **Dependencies**: `launch`, `launch_ros`
   - **Required**: Yes, for any Isaac Sim ROS2 integration

2. **`isaac_ros2_messages/`**
   - **Purpose**: ROS2 message definitions for Isaac Sim
   - **Dependencies**: Standard ROS2 messages
   - **Required**: Yes, if using Isaac Sim messages

3. **`isaac_tutorials/`**
   - **Purpose**: Example code and tutorials
   - **Dependencies**: Various (check package.xml)
   - **Required**: Recommended for learning and reference

### Optional Packages (Add as Needed)

4. **`moveit/`**
   - **Purpose**: MoveIt integration for motion planning
   - **Dependencies**: MoveIt, submodules (moveit_resources, topic_based_ros2_control)
   - **Required**: Only if using MoveIt for planning
   - **Note**: Requires submodule initialization

5. **`navigation/`**
   - **Purpose**: Navigation stack integration
   - **Dependencies**: Nav2, various navigation packages
   - **Required**: Only if using mobile robot navigation

6. **`ackermann_control/`**
   - **Purpose**: Ackermann steering control
   - **Dependencies**: Control packages
   - **Required**: Only for wheeled robots with Ackermann steering

7. **`humanoid_locomotion_policy_example/`**
   - **Purpose**: Humanoid locomotion examples
   - **Dependencies**: Humanoid-specific packages
   - **Required**: Only for humanoid robot development

8. **`custom_message/`**
   - **Purpose**: Custom message definitions
   - **Dependencies**: Standard ROS2 message generation
   - **Required**: Only if using custom messages defined here

---

## Step-by-Step Implementation

### Step 1: Initial Setup

```bash
# 1. Navigate to project root
cd /home/ws

# 2. Add submodule
git submodule add https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces.git IsaacSim-ros_workspaces

# 3. Initialize submodule
git submodule update --init --recursive
```

### Step 2: Configure Sparse Checkout

```bash
cd IsaacSim-ros_workspaces
git config core.sparseCheckout true

# Create sparse-checkout configuration
cat > .git/info/sparse-checkout << 'EOF'
jazzy_ws/*
build_ros.sh
.gitmodules
EOF

# Apply configuration
git read-tree -mu HEAD
cd ..
```

### Step 3: Initialize MoveIt Submodules (if needed)

```bash
cd IsaacSim-ros_workspaces
git submodule update --init --recursive jazzy_ws/src/moveit/moveit_resources
git submodule update --init --recursive jazzy_ws/src/moveit/topic_based_ros2_control
cd ..
```

### Step 4: Integrate Packages into Workspace

**Option: Symlink essential packages**
```bash
# Create symlinks for essential packages
cd src
ln -s ../../IsaacSim-ros_workspaces/jazzy_ws/src/isaacsim isaacsim
ln -s ../../IsaacSim-ros_workspaces/jazzy_ws/src/isaac_ros2_messages isaac_ros2_messages
ln -s ../../IsaacSim-ros_workspaces/jazzy_ws/src/isaac_tutorials isaac_tutorials
cd ..
```

**Option: Copy packages (if you want to modify them)**
```bash
# Copy packages
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaacsim src/
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaac_ros2_messages src/
cp -r IsaacSim-ros_workspaces/jazzy_ws/src/isaac_tutorials src/
```

### Step 5: Update DevContainer (if needed)

Add submodule initialization to `postCreateCommand`:
```json
"postCreateCommand": "sudo rosdep update && cd IsaacSim-ros_workspaces && git submodule update --init --recursive && cd .. && sudo rosdep install --from-paths src --ignore-src -y && sudo chown -R $(whoami) /home/ws/ && if [ -d ~/.ssh ]; then chmod 700 ~/.ssh && chmod 600 ~/.ssh/* 2>/dev/null || true && chmod 644 ~/.ssh/*.pub 2>/dev/null || true; fi"
```

### Step 6: Install Dependencies

```bash
# Update rosdep database
rosdep update

# Install dependencies for Isaac packages
rosdep install --from-paths src --ignore-src -y
```

### Step 7: Build Workspace

```bash
# Build with symlink install (recommended for development)
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Step 8: Verify Installation

```bash
# Check that packages are recognized
ros2 pkg list | grep isaac

# Test launch file (if available)
ros2 launch isaacsim <example_launch_file>.py
```

---

## File Structure After Integration

```
/home/ws/
├── .devcontainer/
│   └── devcontainer.json
├── IsaacSim-ros_workspaces/          # Git submodule
│   ├── .gitmodules
│   ├── build_ros.sh
│   └── jazzy_ws/
│       ├── fastdds.xml
│       └── src/
│           ├── isaacsim/
│           ├── isaac_ros2_messages/
│           ├── isaac_tutorials/
│           ├── moveit/                # With submodules
│           └── ...
├── src/                               # Main workspace
│   ├── isaacsim -> ../IsaacSim-ros_workspaces/jazzy_ws/src/isaacsim  # Symlink
│   ├── isaac_ros2_messages -> ../IsaacSim-ros_workspaces/jazzy_ws/src/isaac_ros2_messages
│   ├── isaac_tutorials -> ../IsaacSim-ros_workspaces/jazzy_ws/src/isaac_tutorials
│   └── <your_custom_packages>/
├── build/
├── install/
├── log/
└── .gitmodules                        # Submodule reference
```

---

## Updating from Upstream

### Updating the Submodule

```bash
# Update to latest commit
cd IsaacSim-ros_workspaces
git pull origin main
cd ..

# Update submodule reference in main repo
git add IsaacSim-ros_workspaces
git commit -m "Update IsaacSim-ros_workspaces submodule"
```

### Updating Submodules Within IsaacSim-ros_workspaces

```bash
cd IsaacSim-ros_workspaces
git submodule update --remote --recursive
cd ..
```

---

## Troubleshooting

### Issue 1: Submodule Not Initialized

**Symptoms:** Missing files in `jazzy_ws/src/moveit/`

**Solution:**
```bash
cd IsaacSim-ros_workspaces
git submodule update --init --recursive
```

### Issue 2: Sparse Checkout Not Working

**Symptoms:** Seeing `humble_ws/` or other unwanted directories

**Solution:**
```bash
cd IsaacSim-ros_workspaces
git read-tree -mu HEAD
# Verify .git/info/sparse-checkout file
cat .git/info/sparse-checkout
```

### Issue 3: Build Errors Due to Missing Dependencies

**Symptoms:** Colcon build fails with missing package errors

**Solution:**
```bash
# Update rosdep
rosdep update

# Install all dependencies
rosdep install --from-paths src --ignore-src -y

# Check specific package dependencies
rosdep check --from-paths src/isaacsim
```

### Issue 4: Symlinks Not Working in Container

**Symptoms:** Packages not found, symlink errors

**Solution:**
- Ensure workspace is properly mounted in devcontainer
- Check that paths are correct (relative vs absolute)
- Consider copying instead of symlinking if issues persist

### Issue 5: FastDDS Configuration Issues

**Symptoms:** DDS communication problems

**Solution:**
```bash
# Copy or symlink fastdds.xml
cp IsaacSim-ros_workspaces/jazzy_ws/fastdds.xml .

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## Decision Points

Before proceeding, determine:

1. **Which packages are needed?**
   - Start with: `isaacsim`, `isaac_ros2_messages`, `isaac_tutorials`
   - Add others as requirements emerge

2. **Integration method?**
   - Recommended: Git submodule with symlinks
   - Alternative: Copy packages if modifications needed

3. **MoveIt integration?**
   - If yes: Initialize MoveIt submodules
   - If no: Can skip MoveIt packages entirely

4. **FastDDS vs default DDS?**
   - Check if `fastdds.xml` is needed
   - May depend on Isaac Sim version requirements

5. **Workspace structure?**
   - Use main `src/` with symlinks (recommended)
   - Or use `jazzy_ws` directly as workspace

---

## Timeline Estimate

| Phase | Duration | Description |
|-------|----------|-------------|
| Setup & Planning | 1 hour | Review packages, decide on approach |
| Submodule Setup | 30 min | Add submodule, configure sparse checkout |
| Package Integration | 1 hour | Symlink/copy packages, update configs |
| Dependency Resolution | 30 min | Install dependencies, resolve issues |
| Build & Test | 1 hour | Build workspace, verify packages work |
| Documentation | 30 min | Update README, document setup |
| **Total** | **~4-5 hours** | Complete integration |

---

## Next Steps

1. **Review this plan** and decide on:
   - Which packages to include
   - Integration method (submodule vs copy)
   - Whether MoveIt is needed

2. **Execute Phase 1** (Repository Setup):
   - Add submodule
   - Configure sparse checkout
   - Initialize submodules

3. **Execute Phase 2** (Workspace Integration):
   - Integrate essential packages
   - Update devcontainer if needed

4. **Test and verify**:
   - Build workspace
   - Test example launch files
   - Verify package discovery

5. **Document**:
   - Update project README
   - Document submodule update process
   - Add setup instructions for team members

---

## Additional Resources

- [Isaac Sim ROS Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
- [Isaac Sim ROS Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Git Submodules Documentation](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
- [ROS2 Workspace Setup](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [IsaacSim-ros_workspaces Repository](https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces)

---

## Questions to Answer

1. **Which Isaac Sim packages are actually needed for your project?**
   - Start minimal (isaacsim, isaac_ros2_messages) or include all?

2. **Will you be using MoveIt?**
   - Determines if MoveIt submodules need initialization

3. **Do you need navigation stack?**
   - Determines if navigation packages should be included

4. **Will you modify Isaac packages?**
   - If yes, copy instead of symlink
   - If no, symlink for easy updates

5. **Team familiarity with git submodules?**
   - May influence choice between submodule vs copy approach

---

*This plan is a living document and should be updated as integration progresses and requirements become clearer.*
