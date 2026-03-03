import paho.mqtt.client as mqtt
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# 🔧 1. Define the current data schema and how it's grouped into plots
# walking
"""data_config = {
    "plot_1": ["LTh_roll","LTh_traj"],
    "plot_2": ["RTh_roll", "RTh_traj"],
    "plot_3": ["LSh_roll", "LSh_traj"],
    "plot_4": ["RTh_gyr"],
    "plot5": ["phase"],
    "text": "phase"  # status text to display
}"""
# stair ascent
"""data_config = {
    "plot_1": ["LTh_gyr"],
    "plot_2": ["LTh_roll", "OTh_roll"],
    "plot_3": ["Lkn_roll", "Okn_roll"],
    "text": "state"  # status text to display
}"""

# sittostand
data_config = {
    "plot_1": ["Lth_gyr"],
    "plot_2": ["Lth_roll", "torso_roll"],
    "plot5": ["phase"],
    "text": "state",  # status text to display
}


# 📦 2. Buffers for each field
history_length = 200
data_buffers = {
    key: deque([0.0] * history_length, maxlen=history_length)
    for group in data_config.values()
    if isinstance(group, list)
    for key in group
}

status_text = ["Waiting for data..."]

# 📈 3. Plot setup (as many subplots as needed)
num_plots = sum(isinstance(v, list) for v in data_config.values())
fig, axs = plt.subplots(num_plots, 1, figsize=(10, 8), sharex=True)

if num_plots == 1:
    axs = [axs]  # ensure it's a list even with one subplot

plot_lines = []
plot_labels = []

for i, (plot_name, keys) in enumerate(data_config.items()):
    if not isinstance(keys, list):
        continue
    for key in keys:
        (line,) = axs[i].plot([], [], label=key)
        plot_lines.append(line)
        plot_labels.append(key)
    axs[i].set_ylabel(", ".join(keys))
    axs[i].legend()
    axs[i].grid(True)

axs[-1].set_xlabel("Time (latest on right)")
status_box = axs[0].text(
    0.02, 0.95, "", transform=axs[0].transAxes, va="top", ha="left"
)


# 📡 4. MQTT message callback
def on_message(client, userdata, msg):
    try:
        message = json.loads(msg.payload.decode())

        # Update buffers dynamically
        for key in data_buffers:
            value = float(message.get(key, 0.0))
            data_buffers[key].append(value)

        # Update status text
        status_value = message.get(data_config.get("text", ""), "")
        status_text[0] = f"State: {status_value}"

    except Exception as e:
        print("Error parsing message:", e)


# 🔄 5. Animation update
def update_plot(frame):
    x = range(history_length)

    # Update lines
    for line, label in zip(plot_lines, plot_labels):
        buffer = data_buffers[label]
        line.set_data(x, buffer)

    # Set y-limits dynamically
    for i, (plot_name, keys) in enumerate(data_config.items()):
        if not isinstance(keys, list):
            continue
        combined = [data_buffers[k] for k in keys]
        all_vals = [v for b in combined for v in b]
        if all_vals:
            axs[i].set_ylim(min(all_vals) - 5, max(all_vals) + 5)
        axs[i].set_xlim(0, history_length)

    # Update status box
    status_box.set_text(status_text[0])
    return plot_lines + [status_box]


# 🚀 6. MQTT setup
client = mqtt.Client()
client.on_message = on_message
client.connect("10.223.82.205", 1883, 60)
client.subscribe("revalexo/data")
client.loop_start()

# 🎬 7. Plot animation loop
ani = animation.FuncAnimation(
    fig, update_plot, interval=5, blit=False, cache_frame_data=False
)

plt.tight_layout()
plt.show()
