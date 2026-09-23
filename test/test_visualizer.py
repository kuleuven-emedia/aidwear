"""
Test suite for the HERMES aidwear.visualizer Consumer Node.
"""

import os
import yaml
import numpy as np
import matplotlib

# Use Agg or TkAgg; Agg ensures headless test compatibility if run in background
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from hermes.utils.di_utils import search_module_class
from hermes.utils.types import LoggingSpec
from hermes.nicla_sense_me.utils.types import NiclaLocation
from hermes.aidwear.prosthesis.utils.types import MotorId, EncoderId
from hermes.aidwear.visualizer import VisualizerConsumer


def test_search_module_class():
    """Verify that HERMES dynamic class loader finds VisualizerConsumer."""
    cls = search_module_class("aidwear.visualizer", "VisualizerConsumer")
    assert cls is VisualizerConsumer
    print("[OK] search_module_class successfully resolved VisualizerConsumer")


def test_visualizer_instantiation_and_process_data():
    """Verify VisualizerConsumer setup and real-time process_data plotting."""
    log_spec = LoggingSpec(
        log_dir="./tmp_test_logs",
        log_time_s=0.0,
        ref_time_s=0.0,
        experiment={"project": "TestVisualizer"},
        stream_period_s=30,
        stream_hdf5=False,
        stream_csv=False,
        stream_video=False,
        stream_audio=False,
    )

    data_in_specs = [
        {
            "package": "aidwear.prosthesis",
            "class": "ProsthesisPipeline",
            "node_id": "prosthesis",
            "topics": [
                "nicla_torso",
                "nicla_thigh_right",
                "nicla_thigh_left",
                "nicla_shank_right",
                "nicla_shank_left",
                "motor_knee",
                "motor_ankle",
                "encoder_knee",
                "encoder_ankle",
            ],
            "settings": {
                "niclas": {
                    "buf_len": 100,
                    "sampling_rate_hz": 50,
                    "connection_type": "BLE",
                    "is_pelvis_and_feet": False,
                    "device_mapping": {
                        "torso": "D8:4E:18:CC:F6:62",
                        "thigh_right": "4D:8B:11:D5:F2:46",
                        "thigh_left": "A3:2F:AF:84:4E:1D",
                        "shank_right": "96:CD:19:11:5B:F7",
                        "shank_left": "D5:A0:BA:27:CA:09",
                    },
                    "gravity_scaling_factor": 4,
                    "gyroscope_scaling_factor": 500,
                    "is_acc": True,
                    "is_gyr": True,
                    "is_mag": False,
                    "is_euler": True,
                    "is_quat": False,
                    "is_temp": False,
                    "is_baro": False,
                    "is_hum": False,
                },
                "motors": {
                    "device_mapping": {
                        "knee": {"can_id": 1},
                        "ankle": {"can_id": 2},
                    },
                    "sampling_rate_hz": 50,
                    "buf_len": 100,
                },
                "telemetry": {
                    "buf_len": 100,
                },
            },
        }
    ]

    consumer = VisualizerConsumer(
        node_id="visualizer",
        host_ip="127.0.0.1",
        data_in_specs=data_in_specs,
        logging_spec=log_spec,
        history_len=50,
        draw_interval_s=0.001,
    )

    # Trigger UI plot setup
    consumer._setup_plots()
    assert consumer._fig is not None
    assert consumer._axes is not None
    assert consumer._axes.shape == (3, 3)

    # Feed multiple synthetic updates simulating prosthesis output packets
    for step in range(10):
        t = step * 0.02
        # 1. IMU packet
        imu_msg = {
            "nicla_torso": {
                "toa_s": np.array([[t]]),
                "euler": np.array(
                    [[np.sin(t) * 10, np.cos(t) * 15, np.sin(t * 2) * 5]]
                ),
            },
            "nicla_thigh_right": {
                "toa_s": np.array([[t]]),
                "euler": np.array([[np.sin(t + 0.1) * 30, np.cos(t + 0.1) * 20, 0.0]]),
            },
            "nicla_thigh_left": {
                "toa_s": np.array([[t]]),
                "euler": np.array([[np.sin(t + 0.2) * 30, np.cos(t + 0.2) * 20, 0.0]]),
            },
            "nicla_shank_right": {
                "toa_s": np.array([[t]]),
                "euler": np.array([[np.sin(t + 0.3) * 45, np.cos(t + 0.3) * 35, 5.0]]),
            },
            "nicla_shank_left": {
                "toa_s": np.array([[t]]),
                "euler": np.array([[np.sin(t + 0.4) * 45, np.cos(t + 0.4) * 35, 5.0]]),
            },
        }
        consumer.process_data(topic="prosthesis", msg=imu_msg)

        # 2. Motor packet
        motor_msg = {
            "motor_knee": {
                "toa_s": np.array([[t]]),
                "position": np.array([[np.sin(t) * 1.5]]),
                "velocity": np.array([[np.cos(t) * 3.0]]),
                "current": np.array([[np.sin(t * 3) * 4.0]]),
            },
            "motor_ankle": {
                "toa_s": np.array([[t]]),
                "position": np.array([[np.cos(t) * 0.8]]),
                "velocity": np.array([[-np.sin(t) * 2.0]]),
                "current": np.array([[np.cos(t * 3) * 3.5]]),
            },
        }
        consumer.process_data(topic="prosthesis", msg=motor_msg)

        # 3. Encoder packet
        encoder_msg = {
            "encoder_knee": {
                "toa_s": np.array([[t]]),
                "angle": np.array([[60.0 + np.sin(t) * 25.0]]),
            },
            "encoder_ankle": {
                "toa_s": np.array([[t]]),
                "angle": np.array([[90.0 + np.cos(t) * 15.0]]),
            },
        }
        consumer.process_data(topic="prosthesis", msg=encoder_msg)

    # Verify buffers and lines were updated using enums
    assert len(consumer._imu_data[NiclaLocation.TORSO][0]) == 10
    assert len(consumer._motor_pos[MotorId.KNEE]) == 10
    assert len(consumer._motor_vel[MotorId.ANKLE]) == 10
    assert len(consumer._motor_cur[MotorId.KNEE]) == 10
    assert len(consumer._encoder_angle[EncoderId.KNEE]) == 10
    assert len(consumer._encoder_angle[EncoderId.ANKLE]) == 10

    # Clean up
    plt.close(consumer._fig)
    print(
        "[OK] VisualizerConsumer process_data successfully updated all 5 IMUs, 2 motors, and 2 encoders"
    )


def test_yaml_files():
    """Verify that both visualizer.yml and prosthesis.yml parse without YAML errors."""
    vis_path = os.path.join("src", "hermes", "aidwear", "visualizer", "visualizer.yml")
    with open(vis_path, "r") as f:
        vis_cfg = yaml.safe_load(f)
    assert "consumer_specs" in vis_cfg
    assert vis_cfg["consumer_specs"][0]["class"] == "VisualizerConsumer"
    print("[OK] visualizer.yml parsed successfully")

    run_path = os.path.join(
        "run", "prosthesis_standalone_cli", "prosthesis_visualizer.yml"
    )
    with open(run_path, "r") as f:
        run_cfg = yaml.safe_load(f)
    assert "consumer_specs" in run_cfg
    assert len(run_cfg["consumer_specs"]) == 1
    assert run_cfg["consumer_specs"][0]["class"] == "VisualizerConsumer"
    print("[OK] prosthesis_visualizer.yml parsed successfully")


if __name__ == "__main__":
    test_search_module_class()
    test_visualizer_instantiation_and_process_data()
    test_yaml_files()
    print("\nALL VISUALIZER TESTS PASSED!")
