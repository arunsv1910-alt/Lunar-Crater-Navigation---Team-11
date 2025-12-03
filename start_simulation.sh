#!/bin/bash
echo "=========================================="
echo "STARTING LUNAR ROVER SIMULATION"
echo "=========================================="

# Set paths
WORLD_FILE="$HOME/lunar_rover_project/worlds/lunar_crater_world.sdf"

# Check if Gazebo Harmonic is installed
if ! command -v gz &> /dev/null; then
    echo "ERROR: Gazebo Harmonic not found!"
    echo "Install with: sudo apt install gz-harmonic"
    exit 1
fi

# Launch Gazebo with lunar world
echo "Launching Gazebo Harmonic with lunar terrain..."
echo "World file: $WORLD_FILE"
echo ""
echo "PRESS 'Ctrl+C' TO STOP THE SIMULATION"
echo "=========================================="

gz sim --verbose 3 "$WORLD_FILE"

echo ""
echo "=========================================="
echo "SIMULATION STOPPED"
echo "=========================================="
