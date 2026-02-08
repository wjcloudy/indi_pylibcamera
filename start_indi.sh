#!/bin/bash

# Cleanup and start INDI Web Manager
# Run this script to ensure a clean INDI environment

# Trap errors so the terminal stays open for debugging
trap 'echo ""; echo "ERROR: Script failed at line $LINENO"; echo "Press Enter to close..."; read' ERR

# Camera kernel module reset — releases stuck hardware
reset_camera_hardware() {
    echo "Resetting camera hardware..."
    # Kill anything holding /dev/video*
    sudo fuser -k /dev/video* 2>/dev/null || true
    sleep 1
    # Reload kernel modules based on platform
    if lsmod | grep -q "pisp_be"; then
        # Pi 5
        sudo rmmod pisp_be 2>/dev/null || true
        sudo rmmod rp1_cfe 2>/dev/null || true
        sleep 1
        sudo modprobe rp1_cfe 2>/dev/null || true
        sudo modprobe pisp_be 2>/dev/null || true
    elif lsmod | grep -q "bcm2835_unicam"; then
        # Pi 4 / earlier
        sudo rmmod bcm2835_unicam 2>/dev/null || true
        sleep 1
        sudo modprobe bcm2835_unicam 2>/dev/null || true
    fi
    sleep 1
    echo "Camera hardware reset complete."
}

# Cleanup function for graceful shutdown
cleanup() {
    echo ""
    echo "Shutting down..."
    # SIGTERM first
    pkill -15 indiserver 2>/dev/null || true
    pkill -15 indi_pylibcamera 2>/dev/null || true
    pkill -15 indi-web 2>/dev/null || true
    sleep 2
    # Force kill anything still around
    pkill -9 indiserver 2>/dev/null || true
    pkill -9 indi_pylibcamera 2>/dev/null || true
    pkill -9 indi-web 2>/dev/null || true
    sleep 1
    # Reset camera hardware so it's clean for next start
    reset_camera_hardware
    echo "All stopped."
    echo "Press Enter to close..."
    read
    exit 0
}

# Catch Ctrl+C and terminal close
trap cleanup SIGINT SIGTERM

echo "=== INDI pylibcamera (wjcloudy) ==="
echo ""

echo "Stopping any existing INDI servers..."

# SIGTERM first — lets drivers release the camera hardware
pkill -15 indiserver 2>/dev/null || true
pkill -15 indi_pylibcamera 2>/dev/null || true
pkill -15 indi-web 2>/dev/null || true
sleep 2

# Force-kill anything still lingering
pkill -9 indiserver 2>/dev/null || true
pkill -9 indi_pylibcamera 2>/dev/null || true
pkill -9 indi-web 2>/dev/null || true
sleep 1

# Check if port 8624 is still in use and kill it
PORT_PID=$(sudo lsof -t -i :8624 2>/dev/null || true)
if [ -n "$PORT_PID" ]; then
    echo "Killing process on port 8624 (PID: $PORT_PID)..."
    sudo kill -15 $PORT_PID 2>/dev/null || true
    sleep 1
    sudo kill -9 $PORT_PID 2>/dev/null || true
fi

# Check if port 7624 (INDI server) is in use
INDI_PID=$(sudo lsof -t -i :7624 2>/dev/null || true)
if [ -n "$INDI_PID" ]; then
    echo "Killing process on port 7624 (PID: $INDI_PID)..."
    sudo kill -15 $INDI_PID 2>/dev/null || true
    sleep 1
    sudo kill -9 $INDI_PID 2>/dev/null || true
fi

# Always reset camera hardware on startup to clear any stale state
reset_camera_hardware

echo "Activating virtual environment..."
source /home/pi/venv_indi_pylibcamera_wjcloudy/bin/activate

echo "Starting INDI Web Manager wjcloudy version on port 8624..."
echo "Access it at: http://$(hostname):8624"
echo "Press Ctrl+C to stop"
echo ""

indi-web -v

# If indi-web exits, keep the terminal open
echo ""
echo "indi-web has stopped."
echo "Press Enter to close..."
read
