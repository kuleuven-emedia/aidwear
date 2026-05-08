#!/bin/sh
# Restart BLE for Niclas to properly connect
# sudo systemctl restart bluetooth
# sudo systemctl restart bluetooth.service
bluetoothctl power off && bluetoothctl power on

# Activate the virtual environment
source .venv/bin/activate
export PYTHONPATH="$(pwd):$PYTHONPATH"

# Auto-increment the trial ID
FILE="./run/ai_fatigue/trial_auto_id.txt"

if [ -f "$FILE" ]; then
    trial_id=$(cat "$FILE")
else
    trial_id=0
fi
trial_id=$((trial_id + 1))
echo "$trial_id" > "$FILE"

# Run the experiment
hermes-cli -o ./data -f ./run/ai_fatigue/ai.yml -e project=RevalexoAiFatigue type=Test trial=$trial_id
