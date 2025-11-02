#!/bin/bash

# Kill rouge make and simv processes
echo "Killing rouge make and simv processes..."

# Kill make processes
pkill -f "make halt.out" 2>/dev/null || true

# Kill simv processes
pkill -f "cpu.simv" 2>/dev/null || true

# Wait a moment
sleep 1

# Check if any are still running
if pgrep -f "make halt.out" > /dev/null || pgrep -f "cpu.simv" > /dev/null; then
    echo "Some processes still running, forcing kill..."
    pkill -9 -f "make halt.out" 2>/dev/null || true
    pkill -9 -f "cpu.simv" 2>/dev/null || true
    sleep 1
fi

echo "Done killing processes."
