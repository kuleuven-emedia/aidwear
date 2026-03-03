import can
import time
import gpiod

from hermes.aidwear.utils.utilities import config_can_linux
from hermes.aidwear.can_control.motor_epos import can_set_torque


if __name__ == "__main__":
    config_can_linux()

    pos = [0] * 10000
    dpos = [0] * 10000
    ddpos = [0] * 10000
    motor_type = "AK10-9"
    motor_type2 = "AK80-8"
    i = 0
    Can0 = can.interface.Bus(channel="can0", interface="socketcan")
    Can1 = can.interface.Bus(channel="can1", interface="socketcan")

    start = time.time()

    T = 1
    # T = [-30]*50
    # T = 5*np.sin(2*np.pi*0.5*np.arange(0,30,0.01))
    pin = 26
    chip = gpiod.Chip("gpiochip4")
    output_line = chip.get_line(pin)
    output_line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)
    output_line.set_value(0)

    time.sleep(3)

    output_line.set_value(1)

    try:
        for torque in T:
            start = time.time()
            while time.time() - start < 3:
                can_set_torque(Can1, 1, torque, motor_type)
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    can_set_torque(Can1, 1, 0, motor_type)

    output_line.set_value(1)
