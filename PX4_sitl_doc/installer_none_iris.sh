#!/bin/bash
set -e

# ==============================
# PX4 SITL (none_iris) Quick Setup
# ==============================

FULL_INSTALL=false

# Parse arguments
if [ "$1" == "--full" ]; then
    FULL_INSTALL=true
fi

echo "[1/5] Updating system..."
sudo apt update

echo "[2/5] Installing core dependencies..."
sudo apt install -y git python3 python3-pip python3-jinja2 python3-nose openjdk-11-jdk

echo "[3/5] Cloning PX4-Autopilot repo..."
if [ ! -d PX4-Autopilot ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi

cd PX4-Autopilot

echo "[4/5] Initializing submodules..."
git submodule update --init --recursive

if [ "$FULL_INSTALL" = true ]; then
    echo "[5/5] Running full PX4 setup (ubuntu.sh)..."
    bash ./Tools/setup/ubuntu.sh
else
    echo "[5/5] Skipping full PX4 setup. (Use --full if you need Gazebo/jMAVSim)"
fi

echo "✅ Setup complete!"
echo "-------------------------------"
echo "To start PX4 SITL (headless, no simulator):"
echo
echo "    cd PX4-Autopilot"
echo "    make px4_sitl_default none_iris"
echo
echo "PX4 SITL will now listen on:"
echo " - UDP 14550 → QGroundControl"
echo " - UDP 14540 → MAVSDK / pymavlink"
echo
