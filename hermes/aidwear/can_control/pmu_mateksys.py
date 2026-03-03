import struct

from ..utils.types import BatteryData


def parse_monitor_message(
    timestamp: float, data: bytes
) -> BatteryData:
    """Parses the CAN frame message into a `BatteryData` object.

    TODO: update the message structure to parse
    Bytes from MatekSys power monitor arrive in ...-endian order.
    [0-1]:
    [2-3]:
    [4-5]:
    [6]:
    [7]:

    Args:
        timestamp (float): Time since epoch when the CAN message was received, where possible, stamped in hardware.
        data (bytes): Decoded bytes from the CAN frame of a motor.

    Returns:
        A BatteryData object representing the battery's state.
    """
    # TODO:
    vol_int16, cur_int16 = struct.unpack(
        ">ii", data
    )

    voltage = ...
    current = ...
    power = voltage * current

    # TODO: expanda battery dataclass with info from the DroneCAN packets 
    return BatteryData(
        timestamp=timestamp,
        voltage=voltage,
        current=current,
        power=power,
    )
