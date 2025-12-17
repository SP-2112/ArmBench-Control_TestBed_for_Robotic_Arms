### Welcome to ARM Bench - Robot Arm Control & Development Toolkit! 🤖

## Quick Start

### Installation
```bash
# Basic installation
cd arm_bench_dev
pip install -e .

# With visualization and control development features
pip install -e ".[visualization]"

# With MATLAB integration
pip install -e ".[visualization,matlab]"

# Everything (all features)
pip install -e ".[dev]"
```

### Basic Commands
```bash
arm-bench version              # Check version
arm-bench --help              # See all commands
arm-bench start               # Launch GUI
arm-bench setup               # Scan hardware
```

## 🆕 New Feature: Control Algorithm Development Studio

Launch the comprehensive control development environment:

```bash
arm-bench matlab-tune
```

**Features:**
- ✅ Define custom control algorithms in Python
- ✅ Configure and tune parameters with bounds
- ✅ Run simulations with multiple plant models
- ✅ Optimize using MATLAB or Python (scipy)
- ✅ Compare multiple parameter sets
- ✅ Real-time performance visualization
- ✅ Save/load configurations
- ✅ Export to ROS2 for hardware deployment

**Documentation:**
- **MATLAB_QUICK_START.md** - Quick 5-minute guide
- **MATLAB_STUDIO.md** - Complete user manual
- **examples/EXAMPLES.md** - 10 detailed examples with pre-built configs

**Example Workflow:**
```bash
# 1. Launch studio
arm-bench matlab-tune

# 2. Load example
# In GUI: Load Config → examples/pd_controller_config.json

# 3. Run simulation
# Click "▶️  Run Simulation"

# 4. Optimize parameters
# Click "🔧 Optimize Parameters"

# 5. View results
# See graphs and optimized parameters

# 6. Deploy to robot
arm-bench create-control position --output my_controller.py
# Edit with optimized parameters
arm-bench deploy-control my_controller.py --mode position
arm-bench launch-control position --freq 100
```

## All Available Commands

```bash
# General
arm-bench version              # Show version info
arm-bench start               # Launch main GUI
arm-bench setup               # Scan and configure hardware

# Visualization
arm-bench visualize           # Launch RViz for robot visualization

# Teleoperation
arm-bench teleop              # Start teleoperation interface

# Workspace Building
arm-bench build-workspace     # Generate Interbotix workspace

# Bimanual Control
arm-bench translate           # Translate between Reactor/Viper configs

# Recording
arm-bench record              # Record RLDS datasets

# Camera Management
arm-bench cameras             # Scan and list cameras

# Custom Control Algorithms
arm-bench create-control      # Create control template
arm-bench deploy-control      # Deploy to ROS2 workspace
arm-bench launch-control      # Launch deployed controller
arm-bench control-info        # Show control documentation

# Control Development
arm-bench matlab-tune         # Launch Control Development Studio
```

## Features Overview

### 1. Hardware Management
- Automatic Dynamixel motor scanning
- Camera detection and configuration
- Motor status monitoring with live updates

### 2. GUI Interface
- Professional dark theme
- Real-time motor information
- Live camera feeds
- Control algorithm selection
- Recording interface

### 3. Control Development
- **Python-based control logic** definition
- **Parameter tuning** with optimization
- **Multiple plant models** for simulation
- **Performance comparison** tools
- **Real-time visualization** with matplotlib
- **MATLAB integration** (optional)

### 4. ROS2 Integration
- Automatic package creation
- Standardized message formats
- Launch file generation
- colcon build integration

### 5. Data Recording
- RLDS dataset format
- Multi-camera support
- Episode management

## Documentation

- **NEW_FEATURES.md** - Quick reference for all new features
- **ENHANCEMENTS.md** - Technical implementation details
- **MATLAB_STUDIO.md** - Complete Control Development Studio guide
- **MATLAB_QUICK_START.md** - 5-minute quick start
- **examples/EXAMPLES.md** - 10 control algorithm examples

## Example: Complete Control Development Workflow

```bash
# 1. Start with GUI
arm-bench start

# 2. Scan hardware
# Select mode and view motor information

# 3. Develop control algorithm
arm-bench matlab-tune
# Define PD controller
# Add parameters: Kp=10, Kd=1
# Run simulation
# Optimize parameters
# Get: Kp=15.23, Kd=2.46

# 4. Create ROS2 controller
arm-bench create-control position --output pd_controller.py

# 5. Edit controller with optimized parameters
# (Edit pd_controller.py with Kp=15.23, Kd=2.46)

# 6. Deploy to ROS2
arm-bench deploy-control pd_controller.py --mode position --freq 100

# 7. Launch on robot
arm-bench launch-control position --freq 100

# 8. Record data
arm-bench record start --episode-name "tuned_pd_test"
# Run controller, collect data
arm-bench record stop
```

## Troubleshooting

### Installation Issues
```bash
# If dependencies fail, create virtual environment
python3 -m venv arm-test
source arm-test/bin/activate  # On macOS/Linux
pip install -e ".[dev]"
```

### Camera Issues
```bash
pip install opencv-python pillow
```

### MATLAB Issues
```bash
# Option 1: Install MATLAB Engine API
pip install matlabengine

# Option 2: Use Python-based optimization (no MATLAB needed)
# In the studio, select "Python Differential Evolution"
```

### ROS2 Issues
```bash
# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/arm_bench_ros_ws/install/setup.bash
```

## Requirements

**Minimum:**
- Python 3.8+
- numpy, typer, pyyaml, rich

**Recommended:**
- matplotlib, scipy (for control development)
- opencv-python, pillow (for camera feeds)
- pygame (for teleoperation)

**Optional:**
- matlabengine (for MATLAB integration)
- ROS2 Humble (for hardware deployment)

## Support

For help:
```bash
arm-bench --help
arm-bench <command> --help
```

**Documentation Files:**
- Quick reference: `NEW_FEATURES.md`
- Control studio: `MATLAB_STUDIO.md`
- Examples: `examples/EXAMPLES.md`

---


### Commands



