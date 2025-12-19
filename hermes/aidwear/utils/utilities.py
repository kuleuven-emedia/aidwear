import os


def config_can_linux():
    os.system("sudo ip link set can0 up type can bitrate 1000000")
    os.system("sudo ifconfig can0 txqueuelen 65536")
    os.system("sudo ip link set can1 up type can bitrate 1000000")
    os.system("sudo ifconfig can1 txqueuelen 65536")


def wrap_angle(x, y):
    return (x + y) % 360


def launch_handler(module, **kwargs) -> None:
    obj = module(**kwargs)
    obj()
