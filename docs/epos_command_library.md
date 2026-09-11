# Maxon EPOS Command Library — Knowledge Summary & Communication Reference
> **Source**: Maxon Motor EPOS Command Library Documentation (rel11757, Document Edition 2023-07)
> **Applies To**: EPOS, EPOS2, EPOS4, IDX drives
> **Communication Protocols Supported**: CANopen, USB, RS232, MAXON SERIAL V2

## 1. Introduction & Communication Architecture
The `EposCmd` library provides a high-level C-interface allowing external controller host applications (PC, embedded Linux/Windows, microcontrollers) to configure, command, and monitor Maxon EPOS positioning controllers.

### Key Operating Modes Supported by EPOS4:
1. **Current / Torque Mode (CM)**: Direct control of motor current/torque setpoints.
2. **Profile Velocity Mode (PVM)**: Velocity control with controlled acceleration and deceleration slopes.
3. **Profile Position Mode (PPM)**: Trajectory generator executing point-to-point trapezoidal or sinusoidal position profiles.
4. **Homing Mode (HM)**: Calibrates zero/home reference position using limit switches, index pulses, or current threshold.
5. **Interpolated Position Mode (IPM)**: Multi-axis coordinated motion through streaming PVT (Position-Velocity-Time) spline interpolation buffers.
6. **Position Mode (PM) & Velocity Mode (VM)**: Fast closed-loop direct setpoint modes (analog or digital).
7. **Master Encoder Mode (MEM) & Step Direction Mode (SDM)**: Electronic gearing and pulse/direction tracking.

### Standard Return Convention:
- Most functions return `BOOL` (`1` = Success / Non-zero, `0` = Failure).
- Handle-generating functions (`VCS_OpenDevice`, `VCS_OpenSubDevice`, etc.) return `HANDLE` (`0` / `NULL` on error).
- All functions take a trailing output pointer `DWORD* pErrorCode`. On failure, `*pErrorCode` is populated with a 32-bit hex error code (detailed in Section 5).

## 2. Data Type Definitions (Manual Section 2.5)
The table below documents all fundamental data types used throughout the EPOS Command Library interface, their sizes in bits and bytes, valid numeric ranges, and their exact mapping to Python `ctypes` bindings.

| Type Name | Description | Size (Bits) | Size (Bytes) | Range | Python `ctypes` Mapping |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `char`, `__int8` | Signed 8-bit integer | 8 | 1 | -128 … 127 | `ctypes.c_int8` / `ctypes.c_byte` |
| `BYTE` | Unsigned 8-bit integer | 8 | 1 | 0 … 255 | `ctypes.c_uint8` / `ctypes.c_ubyte` |
| `short` | Signed 16-bit integer | 16 | 2 | -32,768 … 32,767 | `ctypes.c_int16` / `ctypes.c_short` |
| `WORD` | Unsigned 16-bit integer | 16 | 2 | 0 … 65,535 | `ctypes.c_uint16` / `ctypes.c_ushort` |
| `long`, `int` | Signed 32-bit integer | 32 | 4 | -2,147,483,648 … 2,147,483,647 | `ctypes.c_int32` / `ctypes.c_long` |
| `DWORD` | Unsigned 32-bit integer | 32 | 4 | 0 … 4,294,967,295 | `ctypes.c_uint32` / `ctypes.c_ulong` |
| `DWORD64` | Unsigned 64-bit integer | 64 | 8 | 0 … 18,446,744,073,709,551,615 | `ctypes.c_uint64` / `ctypes.c_ulonglong` |
| `float` | 32-bit single-precision float | 32 | 4 | ±1.18×10⁻³⁸ … ±3.4×10³⁸ | `ctypes.c_float` |
| `BOOL` | Boolean (signed 32-bit int) | 32 | 4 | `1` = TRUE, `0` = FALSE | `ctypes.c_int32` / `ctypes.c_long` |
| `HANDLE` | Object / device handle pointer | 32 / 64 | 4 / 8 | OS pointer address | `ctypes.c_void_p` |
| `char*` | Pointer to null-terminated C string | 32 / 64 | 4 / 8 | String buffer address | `ctypes.c_char_p` |
| `void*` | Generic memory buffer pointer | 32 / 64 | 4 / 8 | Memory buffer address | `ctypes.c_void_p` |
| `T*` (pointer) | Pointer to scalar type `T` (e.g. `WORD*`, `DWORD*`, `long*`) | 32 / 64 | 4 / 8 | Target address | `ctypes.POINTER(ctypes.<T>)` |

## 3. Communication Primitives & Command Signatures
This section catalogues all 201 functions of the EPOS Command Library organized by functional chapter.


---

## Chapter 3: Initialization Functions

### 3.1 Communication Initialization & Port Management

#### `VCS_OpenDevice` (3.1.1)
**Description**: VCS_OpenDevice opens the port to send and receive commands. Ports can be RS232, USB, and CANopen interfaces. For correct designations on DeviceName, ProtocolStackName, InterfaceName, and PortName, use the functions VCS_GetDeviceNameSelection, VCS_GetProtocolStackNameSelection, VCS_GetInterfaceNameSelection, and VCS_GetPortNameSelection. For gateway topologies use function VCS_OpenSubDevice.

```c
HANDLE VCS_OpenDevice(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `HANDLE` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `InterfaceName` | `char*` | Input | Parameter `InterfaceName` |
| `PortName` | `char*` | Input | Parameter `PortName` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_OpenDeviceDlg` (3.1.2)
**Description**: VCS_OpenDeviceDlg recognizes available interfaces capable to operate with EPOS and opens the selected interface for communication. Select “EPOS4” for IDX drives. Not available with Linux.

```c
HANDLE VCS_OpenDeviceDlg(DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `HANDLE` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetProtocolStackSettings` (3.1.3)
**Description**: VCS_SetProtocolStackSettings writes the communication parameters. For exact values on available baud rates, use function VCS_GetBaudRateSelection. For correct communication, use the same baud rate as the connected device. In gateway topologies for subdevice use VCS_SetGatewaySettings instead.

```c
BOOL VCS_SetProtocolStackSettings(HANDLE KeyHandle, DWORD Baudrate, DWORD Timeout, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `Baudrate` | `DWORD` | Input | Parameter `Baudrate` |
| `Timeout` | `DWORD` | Input | Parameter `Timeout` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetProtocolStackSettings` (3.1.4)
**Description**: VCS_GetProtocolStackSettings returns the baud rate and timeout communication parameters. In gateway topologies for subdevice use VCS_GetGatewaySettings instead.

```c
BOOL VCS_GetProtocolStackSettings(HANDLE KeyHandle, DWORD* pBaudrate, DWORD* pTimeout, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pBaudrate` | `DWORD*` | Output | Parameter `pBaudrate` |
| `pTimeout` | `DWORD*` | Output | Parameter `pTimeout` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_FindDeviceCommunicationSettings` (3.1.5)
**Description**: VCS_FindDeviceCommunicationSettings searches the communication setting parameters. Parameters can be defined to accelerate the process. The search will be terminated as the first device is found. Not available with Linux.

```c
BOOL VCS_FindDeviceCommunicationSettings(HANDLE* pKeyHandle, char* pDeviceName, char* pProtocolStackName, char* pInterfaceName, char* pPortName, WORD SizeName, DWORD* pBaudrate, DWORD* pTimeout, WORD* pNodeId, int DialogMode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `pKeyHandle` | `HANDLE*` | Output | Parameter `pKeyHandle` |
| `pDeviceName` | `char*` | Output | Parameter `pDeviceName` |
| `pProtocolStackName` | `char*` | Output | Parameter `pProtocolStackName` |
| `pInterfaceName` | `char*` | Output | Parameter `pInterfaceName` |
| `pPortName` | `char*` | Output | Parameter `pPortName` |
| `SizeName` | `WORD` | Input | Parameter `SizeName` |
| `pBaudrate` | `DWORD*` | Output | Parameter `pBaudrate` |
| `pTimeout` | `DWORD*` | Output | Parameter `pTimeout` |
| `pNodeId` | `WORD*` | Output | Parameter `pNodeId` |
| `DialogMode` | `int` | Input | Parameter `DialogMode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_CloseAllDevices` (3.1.6)
**Description**: VCS_CloseAllDevices closes all opened ports for devices and subdevices and releases them for other applications. If no opened ports are available, the function returns "0".

```c
BOOL VCS_CloseAllDevices(DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_CloseDevice` (3.1.7)
**Description**: VCS_CloseDevice closes the port and releases it for other applications. If no opened ports are available, the function returns “0”.

```c
BOOL VCS_CloseDevice(HANDLE KeyHandle, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_OpenSubDevice` (3.1.8)
**Description**: VCS_OpenSubDevice opens the subdevice connected to the gateway device to send and receive commands.

```c
HANDLE VCS_OpenSubDevice(HANDLE DeviceHandle, char* DeviceName, char* ProtocolStackName, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `HANDLE` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceHandle` | `HANDLE` | Input | Device key handle |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_OpenSubDeviceDlg` (3.1.9)
**Description**: VCS_OpenSubDeviceDlg recognizes available subdevices capable to operate with the gateway device and opens the selected device for communication. Select “EPOS4” for IDX drives. Not available with Linux.

```c
HANDLE VCS_OpenSubDeviceDlg(HANDLE DeviceHandle, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `HANDLE` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceHandle` | `HANDLE` | Input | Device key handle |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetGatewaySettings` (3.1.10)
**Description**: VCS_SetGatewaySettings writes the gateway communication parameters to the device, stores them, and resets the gateway device. The function does not set the communication parameters to all devices on the bus. For correct communication, use the same baud rate as the connected devices.

```c
BOOL VCS_SetGatewaySettings(HANDLE KeyHandle, DWORD Baudrate, WORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `Baudrate` | `DWORD` | Input | Parameter `Baudrate` |
| `pErrorCode` | `WORD*` | Output | Pointer to error code buffer |

#### `VCS_GetGatewaySettings` (3.1.11)
**Description**: VCS_GetGatewaySettings returns the baud rate gateway communication parameter.

```c
BOOL VCS_GetGatewaySettings(HANDLE KeyHandle, DWORD* pBaudrate, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pBaudrate` | `DWORD*` | Output | Parameter `pBaudrate` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_FindSubDeviceCommunicationSettings` (3.1.12)
**Description**: VCS_FindSubDeviceCommunicationSettings searches the subdevice communication setting parameters. The parameters can be defined to accelerate the process. The search will be terminated as the first device is found. Not available with Linux.

```c
BOOL VCS_FindSubDeviceCommunicationSettings(HANDLE DeviceHandle, HANDLE* pKeyHandle, char* pDeviceName, char* pProtocolStackName, WORD SizeName, DWORD* pBaudrate, WORD* pNodeId, int DialogMode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceHandle` | `HANDLE` | Input | Device key handle |
| `pKeyHandle` | `HANDLE*` | Output | Parameter `pKeyHandle` |
| `pDeviceName` | `char*` | Output | Parameter `pDeviceName` |
| `pProtocolStackName` | `char*` | Output | Parameter `pProtocolStackName` |
| `SizeName` | `WORD` | Input | Parameter `SizeName` |
| `pBaudrate` | `DWORD*` | Output | Parameter `pBaudrate` |
| `pNodeId` | `WORD*` | Output | Parameter `pNodeId` |
| `DialogMode` | `int` | Input | Parameter `DialogMode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_CloseAllSubDevices` (3.1.13)
**Description**: VCS_CloseAllSubDevices closes all opened subdevices and releases them for other applications.

```c
BOOL VCS_CloseAllSubDevices(HANDLE DeviceHandle, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceHandle` | `HANDLE` | Input | Device key handle |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_CloseSubDevice` (3.1.14)
**Description**: VCS_CloseSubDevice closes the subdevice and releases it for other applications.

```c
BOOL VCS_CloseSubDevice(HANDLE KeyHandle, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 3.2 Library & Device Info

#### `VCS_GetErrorInfo` (3.2.1)
**Description**: VCS_GetErrorInfo returns the error information on the executed function from a received error code. It returns communication and library errors. For error codes chapter "8 Error Overview" on page 8-147.

```c
BOOL VCS_GetErrorInfo(DWORD ErrorCodeValue, char* pErrorInfo, WORD MaxStrSize)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `ErrorCodeValue` | `DWORD` | Input | Parameter `ErrorCodeValue` |
| `pErrorInfo` | `char*` | Output | Parameter `pErrorInfo` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |

#### `VCS_GetDriverInfo` (3.2.2)
**Description**: VCS_GetDriverInfo returns the name and version from the "EPOS Command Library".

```c
BOOL VCS_GetDriverInfo(char* pLibraryName, WORD MaxStrNameSize, char* pLibraryVersion, WORD MaxStrVersionSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `pLibraryName` | `char*` | Output | Parameter `pLibraryName` |
| `MaxStrNameSize` | `WORD` | Input | Parameter `MaxStrNameSize` |
| `pLibraryVersion` | `char*` | Output | Parameter `pLibraryVersion` |
| `MaxStrVersionSize` | `WORD` | Input | Parameter `MaxStrVersionSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetVersion` (3.2.3)
**Description**: VCS_GetVersion returns the firmware version.

```c
BOOL VCS_GetVersion(HANDLE KeyHandle, WORD NodeId, WORD* pHardwareVersion, WORD* pSoftwareVersion, WORD* pApplicationNumber, WORD* pApplicationVersion, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pHardwareVersion` | `WORD*` | Output | Parameter `pHardwareVersion` |
| `pSoftwareVersion` | `WORD*` | Output | Parameter `pSoftwareVersion` |
| `pApplicationNumber` | `WORD*` | Output | Parameter `pApplicationNumber` |
| `pApplicationVersion` | `WORD*` | Output | Parameter `pApplicationVersion` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 3.3 Advanced Device & Interface Selection

#### `VCS_GetDeviceNameSelection` (3.3.1)
**Description**: VCS_GetDeviceNameSelection returns all available device names.

```c
BOOL VCS_GetDeviceNameSelection(BOOL StartOfSelection, char* pDeviceNameSel, WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `StartOfSelection` | `BOOL` | Input | Parameter `StartOfSelection` |
| `pDeviceNameSel` | `char*` | Output | Parameter `pDeviceNameSel` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pEndOfSelection` | `BOOL*` | Output | Parameter `pEndOfSelection` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetProtocolStackNameSelection` (3.3.2)
**Description**: VCS_GetProtocolStackNameSelection returns all available protocol stack names.

```c
BOOL VCS_GetProtocolStackNameSelection(char* DeviceName, BOOL StartOfSelection, char* pProtocolStackNameSel, WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `StartOfSelection` | `BOOL` | Input | Parameter `StartOfSelection` |
| `pProtocolStackNameSel` | `char*` | Output | Parameter `pProtocolStackNameSel` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pEndOfSelection` | `BOOL*` | Output | Parameter `pEndOfSelection` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetInterfaceNameSelection` (3.3.3)
**Description**: VCS_GetInterfaceNameSelection returns all available interface names.

```c
BOOL VCS_GetInterfaceNameSelection(char* DeviceName, char* ProtocolStackName, BOOL StartOfSelection, char* pInterfaceNameSel, WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `StartOfSelection` | `BOOL` | Input | Parameter `StartOfSelection` |
| `pInterfaceNameSel` | `char*` | Output | Parameter `pInterfaceNameSel` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pEndOfSelection` | `BOOL*` | Output | Parameter `pEndOfSelection` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPortNameSelection` (3.3.4)
**Description**: VCS_GetPortNameSelection returns all available port names.

```c
BOOL VCS_GetPortNameSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, BOOL StartOfSelection, char* pPortSel, WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `InterfaceName` | `char*` | Input | Parameter `InterfaceName` |
| `StartOfSelection` | `BOOL` | Input | Parameter `StartOfSelection` |
| `pPortSel` | `char*` | Output | Parameter `pPortSel` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pEndOfSelection` | `BOOL*` | Output | Parameter `pEndOfSelection` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ResetPortNameSelection` (3.3.5)
**Description**: VCS_ResetPortNameSelection reinitializes the port enumeration.

```c
BOOL VCS_ResetPortNameSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `InterfaceName` | `char*` | Input | Parameter `InterfaceName` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetBaudrateSelection` (3.3.6)
**Description**: VCS_GetBaudrateSelection returns all available baud rates for the connected port.

```c
BOOL VCS_GetBaudrateSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, BOOL StartOfSelection, DWORD* pBaudrateSel, BOOL* pEndOfSelection, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `InterfaceName` | `char*` | Input | Parameter `InterfaceName` |
| `PortName` | `char*` | Input | Parameter `PortName` |
| `StartOfSelection` | `BOOL` | Input | Parameter `StartOfSelection` |
| `pBaudrateSel` | `DWORD*` | Output | Parameter `pBaudrateSel` |
| `pEndOfSelection` | `BOOL*` | Output | Parameter `pEndOfSelection` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetKeyHandle` (3.3.7)
**Description**: VCS_GetKeyHandle returns the key handle from the opened interface.

```c
BOOL VCS_GetKeyHandle(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, HANDLE* pKeyHandle, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `DeviceName` | `char*` | Input | Parameter `DeviceName` |
| `ProtocolStackName` | `char*` | Input | Parameter `ProtocolStackName` |
| `InterfaceName` | `char*` | Input | Parameter `InterfaceName` |
| `PortName` | `char*` | Input | Parameter `PortName` |
| `pKeyHandle` | `HANDLE*` | Output | Parameter `pKeyHandle` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetDeviceName` (3.3.8)
**Description**: VCS_GetDeviceName returns the device name to corresponding handle.

```c
BOOL VCS_GetDeviceName(HANDLE KeyHandle, char* pDeviceName, WORD MaxStrSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pDeviceName` | `char*` | Output | Parameter `pDeviceName` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetProtocolStackName` (3.3.9)
**Description**: VCS_GetProtocolStackName returns the protocol stack name to corresponding handle.

```c
BOOL VCS_GetProtocolStackName(HANDLE KeyHandle, char* pProtocolStackName, WORD MaxStrSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pProtocolStackName` | `char*` | Output | Parameter `pProtocolStackName` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetInterfaceName` (3.3.10)
**Description**: VCS_GetInterfaceName returns the interface name to corresponding handle.

```c
BOOL VCS_GetInterfaceName(HANDLE KeyHandle, char* pInterfaceName, WORD MaxStrSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pInterfaceName` | `char*` | Output | Parameter `pInterfaceName` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPortName` (3.3.11)
**Description**: VCS_GetPortName returns the port name to corresponding handle.

```c
BOOL VCS_GetPortName(HANDLE KeyHandle, char* pPortName, WORD MaxStrSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `pPortName` | `char*` | Output | Parameter `pPortName` |
| `MaxStrSize` | `WORD` | Input | Parameter `MaxStrSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |


---

## Chapter 4: Configuration Functions

### 4.1 General Configuration & Object Dictionary Access

#### `VCS_ImportParameter` (4.1.1)
**Description**: VCS_ImportParameter writes parameters from a file to the device. Not available with Linux.

```c
BOOL VCS_ImportParameter(HANDLE KeyHandle, WORD NodeId, char* pParameterFileName, BOOL ShowDlg, BOOL ShowMsg, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pParameterFileName` | `char*` | Output | Parameter `pParameterFileName` |
| `ShowDlg` | `BOOL` | Input | Parameter `ShowDlg` |
| `ShowMsg` | `BOOL` | Input | Parameter `ShowMsg` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ExportParameter` (4.1.2)
**Description**: VCS_ExportParameter reads all device parameters and writes them to the file. Not available with Linux.

```c
BOOL VCS_ExportParameter(HANDLE KeyHandle, WORD NodeId, char* pParameterFileName, char* pFirmwareFileName, char* pUserID, char* pComment, BOOL ShowDlg, BOOL ShowMsg, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pParameterFileName` | `char*` | Output | Parameter `pParameterFileName` |
| `pFirmwareFileName` | `char*` | Output | Parameter `pFirmwareFileName` |
| `pUserID` | `char*` | Output | Parameter `pUserID` |
| `pComment` | `char*` | Output | Parameter `pComment` |
| `ShowDlg` | `BOOL` | Input | Parameter `ShowDlg` |
| `ShowMsg` | `BOOL` | Input | Parameter `ShowMsg` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetObject` (4.1.3)
**Description**: VCS_SetObject writes an object value at the given index and subindex. For information on object index, object subindex, and object length see separate document «Firmware Specification».

```c
BOOL VCS_SetObject(HANDLE KeyHandle, WORD NodeId, WORD ObjectIndex, BYTE ObjectSubIndex, void* pData, DWORD NbOfBytesToWrite, DWORD* pNbOfBytesWritten, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ObjectIndex` | `WORD` | Input | Parameter `ObjectIndex` |
| `ObjectSubIndex` | `BYTE` | Input | Parameter `ObjectSubIndex` |
| `pData` | `void*` | Output | Parameter `pData` |
| `NbOfBytesToWrite` | `DWORD` | Input | Parameter `NbOfBytesToWrite` |
| `pNbOfBytesWritten` | `DWORD*` | Output | Parameter `pNbOfBytesWritten` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetObject` (4.1.4)
**Description**: VCS_GetObject reads an object value at the given index and subindex. For information on object index, object subindex, and object length see separate document "Firmware Specification".

```c
BOOL VCS_GetObject(HANDLE KeyHandle, WORD NodeId, WORD ObjectIndex, BYTE ObjectSubIndex, void* pData, DWORD NbOfBytesToRead, DWORD* pNbOfBytesRead, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ObjectIndex` | `WORD` | Input | Parameter `ObjectIndex` |
| `ObjectSubIndex` | `BYTE` | Input | Parameter `ObjectSubIndex` |
| `pData` | `void*` | Output | Parameter `pData` |
| `NbOfBytesToRead` | `DWORD` | Input | Parameter `NbOfBytesToRead` |
| `pNbOfBytesRead` | `DWORD*` | Output | Parameter `pNbOfBytesRead` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_Restore` (4.1.5)
**Description**: VCS_Restore restores all default parameters.

```c
BOOL VCS_Restore(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_Store` (4.1.6)
**Description**: VCS_Store stores all parameters.

```c
BOOL VCS_Store(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_UpdateFirmware` (4.1.7)
**Description**: VCS_UpdateFirmware is used to update the binary code for the controller firmware. Not available with Linux.

```c
BOOL VCS_UpdateFirmware (HANDLE KeyHandle, WORD NodeId, char *pBinaryFile, BOOL ShowDlg, BOOL ShowHistory, BOOL ShowMsg, DWORD *pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pBinaryFile` | `char*` | Output | Parameter `pBinaryFile` |
| `ShowDlg` | `BOOL` | Input | Parameter `ShowDlg` |
| `ShowHistory` | `BOOL` | Input | Parameter `ShowHistory` |
| `ShowMsg` | `BOOL` | Input | Parameter `ShowMsg` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 4.2 Advanced Motor, Sensor, Safety & Controller Configuration

#### `VCS_SetMotorType` (4.2.1.1)
**Description**: VCS_SetMotorType writes the motor type.

```c
BOOL VCS_SetMotorType(HANDLE KeyHandle, WORD NodeId, WORD MotorType, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `MotorType` | `WORD` | Input | Parameter `MotorType` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetDcMotorParameter` (4.2.1.2)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_SetDcMotorParameterEx instead.

**Description**: VCS_SetDcMotorParameter writes all DC motor parameters.

```c
BOOL VCS_SetDcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD NominalCurrent, WORD MaxOutputCurrent, WORD ThermalTimeConstant, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `NominalCurrent` | `WORD` | Input | Parameter `NominalCurrent` |
| `MaxOutputCurrent` | `WORD` | Input | Parameter `MaxOutputCurrent` |
| `ThermalTimeConstant` | `WORD` | Input | Parameter `ThermalTimeConstant` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetDcMotorParameterEx` (4.2.1.3)
**Description**: VCS_SetDcMotorParameterEx writes all DC motor parameters.

```c
BOOL VCS_SetDcMotorParameterEx(HANDLE KeyHandle, WORD NodeId, DWORD NominalCurrent, DWORD MaxOutputCurrent, WORD ThermalTimeConstant, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `NominalCurrent` | `DWORD` | Input | Parameter `NominalCurrent` |
| `MaxOutputCurrent` | `DWORD` | Input | Parameter `MaxOutputCurrent` |
| `ThermalTimeConstant` | `WORD` | Input | Parameter `ThermalTimeConstant` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetEcMotorParameter` (4.2.1.4)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_SetEcMotorParameterEx instead.

**Description**: VCS_SetEcMotorParameter writes all EC motor parameters.

```c
BOOL VCS_SetEcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD NominalCurrent, WORD MaxOutputCurrent, WORD ThermalTimeConstant, BYTE NbOfPolePairs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `NominalCurrent` | `WORD` | Input | Parameter `NominalCurrent` |
| `MaxOutputCurrent` | `WORD` | Input | Parameter `MaxOutputCurrent` |
| `ThermalTimeConstant` | `WORD` | Input | Parameter `ThermalTimeConstant` |
| `NbOfPolePairs` | `BYTE` | Input | Parameter `NbOfPolePairs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetEcMotorParameterEx` (4.2.1.5)
**Description**: VCS_SetEcMotorParameterEx writes all EC motor parameters.

```c
BOOL VCS_SetEcMotorParameterEx(HANDLE KeyHandle, WORD NodeId, DWORD NominalCurrent, DWORD MaxOutputCurrent, WORD ThermalTimeConstant, BYTE NbOfPolePairs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `NominalCurrent` | `DWORD` | Input | Parameter `NominalCurrent` |
| `MaxOutputCurrent` | `DWORD` | Input | Parameter `MaxOutputCurrent` |
| `ThermalTimeConstant` | `WORD` | Input | Parameter `ThermalTimeConstant` |
| `NbOfPolePairs` | `BYTE` | Input | Parameter `NbOfPolePairs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetMotorType` (4.2.1.6)
**Description**: VCS_GetMotorType reads the motor type.

```c
BOOL VCS_GetMotorType(HANDLE KeyHandle, WORD NodeId, WORD* pMotorType, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pMotorType` | `WORD*` | Output | Parameter `pMotorType` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetDcMotorParameter` (4.2.1.7)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_GetDcMotorParameterEx instead.

**Description**: VCS_GetDcMotorParameter reads all DC motor parameters.

```c
BOOL VCS_GetDcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD* pNominalCurrent, WORD* pMaxOutputCurrent, WORD* pThermalTimeConstant, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pNominalCurrent` | `WORD*` | Output | Parameter `pNominalCurrent` |
| `pMaxOutputCurrent` | `WORD*` | Output | Parameter `pMaxOutputCurrent` |
| `pThermalTimeConstant` | `WORD*` | Output | Parameter `pThermalTimeConstant` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetDcMotorParameterEx` (4.2.1.8)
**Description**: VCS_GetDcMotorParameterEx reads all DC motor parameters.

```c
BOOL VCS_GetDcMotorParameterEx(HANDLE KeyHandle, WORD NodeId, DWORD* pNominalCurrent, DWORD* pMaxOutputCurrent, WORD* pThermalTimeConstant, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pNominalCurrent` | `DWORD*` | Output | Parameter `pNominalCurrent` |
| `pMaxOutputCurrent` | `DWORD*` | Output | Parameter `pMaxOutputCurrent` |
| `pThermalTimeConstant` | `WORD*` | Output | Parameter `pThermalTimeConstant` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetEcMotorParameter` (4.2.1.9)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_GetEcMotorParameterEx instead.

**Description**: VCS_GetEcMotorParameter reads all EC motor parameters.

```c
BOOL VCS_GetEcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD* pNominalCurrent, WORD* pMaxOutputCurrent, WORD* pThermalTimeConstant, BYTE* pNbOfPolePairs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pNominalCurrent` | `WORD*` | Output | Parameter `pNominalCurrent` |
| `pMaxOutputCurrent` | `WORD*` | Output | Parameter `pMaxOutputCurrent` |
| `pThermalTimeConstant` | `WORD*` | Output | Parameter `pThermalTimeConstant` |
| `pNbOfPolePairs` | `BYTE*` | Output | Parameter `pNbOfPolePairs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetEcMotorParameterEx` (4.2.1.10)
**Description**: VCS_GetEcMotorParameterEx reads all EC motor parameters.

```c
BOOL VCS_GetEcMotorParameterEx(HANDLE KeyHandle, WORD NodeId, DWORD* pNominalCurrent, DWORD* pMaxOutputCurrent, WORD* pThermalTimeConstant, BYTE* pNbOfPolePairs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pNominalCurrent` | `DWORD*` | Output | Parameter `pNominalCurrent` |
| `pMaxOutputCurrent` | `DWORD*` | Output | Parameter `pMaxOutputCurrent` |
| `pThermalTimeConstant` | `WORD*` | Output | Parameter `pThermalTimeConstant` |
| `pNbOfPolePairs` | `BYTE*` | Output | Parameter `pNbOfPolePairs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetSensorType` (4.2.2.1)
**Description**: VCS_SetSensorType writes the sensor type.

```c
BOOL VCS_SetSensorType(HANDLE KeyHandle, WORD NodeId, WORD SensorType, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `SensorType` | `WORD` | Input | Parameter `SensorType` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetIncEncoderParameter` (4.2.2.2)
**Description**: VCS_SetIncEncoderParameter writes the incremental encoder parameters.

```c
BOOL VCS_SetIncEncoderParameter(HANDLE KeyHandle, WORD NodeId, DWORD EncoderResolution, BOOL InvertedPolarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `EncoderResolution` | `DWORD` | Input | Parameter `EncoderResolution` |
| `InvertedPolarity` | `BOOL` | Input | Parameter `InvertedPolarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetHallSensorParameter` (4.2.2.3)
**Description**: VCS_SetHallSensorParameter writes the Hall sensor parameter.

```c
BOOL VCS_SetHallSensorParameter(HANDLE KeyHandle, WORD NodeId, BOOL InvertedPolarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `InvertedPolarity` | `BOOL` | Input | Parameter `InvertedPolarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetSsiAbsEncoderParameter` (4.2.2.4)
**Description**: VCS_SetSsiAbsEncoderParameter writes all parameters for SSI absolute encoder.

```c
BOOL VCS_SetSsiAbsEncoderParameter(HANDLE KeyHandle, WORD NodeId, WORD DataRate, WORD NbOfMultiTurnDataBits, WORD NbOfSingleTurnDataBits, BOOL InvertedPolarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DataRate` | `WORD` | Input | Parameter `DataRate` |
| `NbOfMultiTurnDataBits` | `WORD` | Input | Parameter `NbOfMultiTurnDataBits` |
| `NbOfSingleTurnDataBits` | `WORD` | Input | Parameter `NbOfSingleTurnDataBits` |
| `InvertedPolarity` | `BOOL` | Input | Parameter `InvertedPolarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetSsiAbsEncoderParameterEx` (4.2.2.5)
**Description**: VCS_SetSsiAbsEncoderParameterEx writes all parameters for EPOS4 SSI absolute encoder.

```c
BOOL VCS_SetSsiAbsEncoderParameterEx(HANDLE KeyHandle, WORD NodeId, WORD DataRate, WORD NbOfMultiTurnDataBits, WORD NbOfSingleTurnDataBits, WORD NbOfSpecialDataBits, BOOL InvertedPolarity, WORD Timeout, WORD PowerupTime, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DataRate` | `WORD` | Input | Parameter `DataRate` |
| `NbOfMultiTurnDataBits` | `WORD` | Input | Parameter `NbOfMultiTurnDataBits` |
| `NbOfSingleTurnDataBits` | `WORD` | Input | Parameter `NbOfSingleTurnDataBits` |
| `NbOfSpecialDataBits` | `WORD` | Input | Parameter `NbOfSpecialDataBits` |
| `InvertedPolarity` | `BOOL` | Input | Parameter `InvertedPolarity` |
| `Timeout` | `WORD` | Input | Parameter `Timeout` |
| `PowerupTime` | `WORD` | Input | Parameter `PowerupTime` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetSsiAbsEncoderParameterEx2` (4.2.2.6)
**Description**: VCS_SetSsiAbsEncoderParameterEx2 writes all parameters for EPOS4 SSI absolute encoder.

```c
BOOL VCS_SetSsiAbsEncoderParameterEx2(HANDLE KeyHandle, WORD NodeId, WORD DataRate, WORD NbOfSpecialDataBitsLeading, WORD NbOfMultiTurnDataBits, WORD NbOfMultiTurnPositionBits, WORD NbOfSingleTurnDataBits, WORD NbOfSingleTurnPositionBits, WORD NbOfSpecialDataBitsTrailing, BOOL InvertedPolarity, WORD Timeout, WORD PowerupTime, BOOL CheckFrame, BOOL ReferenceReset, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DataRate` | `WORD` | Input | Parameter `DataRate` |
| `NbOfSpecialDataBitsLeading` | `WORD` | Input | Parameter `NbOfSpecialDataBitsLeading` |
| `NbOfMultiTurnDataBits` | `WORD` | Input | Parameter `NbOfMultiTurnDataBits` |
| `NbOfMultiTurnPositionBits` | `WORD` | Input | Parameter `NbOfMultiTurnPositionBits` |
| `NbOfSingleTurnDataBits` | `WORD` | Input | Parameter `NbOfSingleTurnDataBits` |
| `NbOfSingleTurnPositionBits` | `WORD` | Input | Parameter `NbOfSingleTurnPositionBits` |
| `NbOfSpecialDataBitsTrailing` | `WORD` | Input | Parameter `NbOfSpecialDataBitsTrailing` |
| `InvertedPolarity` | `BOOL` | Input | Parameter `InvertedPolarity` |
| `Timeout` | `WORD` | Input | Parameter `Timeout` |
| `PowerupTime` | `WORD` | Input | Parameter `PowerupTime` |
| `CheckFrame` | `BOOL` | Input | Parameter `CheckFrame` |
| `ReferenceReset` | `BOOL` | Input | Parameter `ReferenceReset` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetSensorType` (4.2.2.7)
**Description**: VCS_GetSensorType reads the sensor type.

```c
BOOL VCS_GetSensorType(HANDLE KeyHandle, WORD NodeId, WORD* pSensorType, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pSensorType` | `WORD*` | Output | Parameter `pSensorType` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetIncEncoderParameter` (4.2.2.8)
**Description**: VCS_GetIncEncoderParameter reads the incremental encoder parameters.

```c
BOOL VCS_GetIncEncoderParameter(HANDLE KeyHandle, WORD NodeId, DWORD* pEncoderResolution, BOOL* pInvertedPolarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pEncoderResolution` | `DWORD*` | Output | Parameter `pEncoderResolution` |
| `pInvertedPolarity` | `BOOL*` | Output | Parameter `pInvertedPolarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetHallSensorParameter` (4.2.2.9)
**Description**: VCS_GetHallSensorParameter reads the Hall sensor parameters.

```c
BOOL VCS_GetHallSensorParameter(HANDLE KeyHandle, WORD NodeId, BOOL* pInvertedPolarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pInvertedPolarity` | `BOOL*` | Output | Parameter `pInvertedPolarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetSsiAbsEncoderParameter` (4.2.2.10)
**Description**: VCS_GetSsiAbsEncoderParameter reads all parameters from SSI absolute encoder.

```c
BOOL VCS_GetSsiAbsEncoderParameter(HANDLE KeyHandle, WORD NodeId, WORD* pDataRate, WORD* pNbOfMultiTurnDataBits, WORD* pNbOfSingleTurnDataBits, BOOL* pInvertedPolarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pDataRate` | `WORD*` | Output | Parameter `pDataRate` |
| `pNbOfMultiTurnDataBits` | `WORD*` | Output | Parameter `pNbOfMultiTurnDataBits` |
| `pNbOfSingleTurnDataBits` | `WORD*` | Output | Parameter `pNbOfSingleTurnDataBits` |
| `pInvertedPolarity` | `BOOL*` | Output | Parameter `pInvertedPolarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetSsiAbsEncoderParameterEx` (4.2.2.11)
**Description**: VCS_GetSsiAbsEncoderParameterEx reads all parameters from EPOS4 SSI absolute encoder.

```c
BOOL VCS_GetSsiAbsEncoderParameterEx(HANDLE KeyHandle, WORD NodeId, WORD* pDataRate, WORD* pNbOfMultiTurnDataBits, WORD* pNbOfSingleTurnDataBits, WORD* pNbOfSpecialDataBits, BOOL* pInvertedPolarity, WORD* pTimeout, WORD* pPowerupTime, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pDataRate` | `WORD*` | Output | Parameter `pDataRate` |
| `pNbOfMultiTurnDataBits` | `WORD*` | Output | Parameter `pNbOfMultiTurnDataBits` |
| `pNbOfSingleTurnDataBits` | `WORD*` | Output | Parameter `pNbOfSingleTurnDataBits` |
| `pNbOfSpecialDataBits` | `WORD*` | Output | Parameter `pNbOfSpecialDataBits` |
| `pInvertedPolarity` | `BOOL*` | Output | Parameter `pInvertedPolarity` |
| `pTimeout` | `WORD*` | Output | Parameter `pTimeout` |
| `pPowerupTime` | `WORD*` | Output | Parameter `pPowerupTime` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetSsiAbsEncoderParameterEx2` (4.2.2.12)
**Description**: VCS_GetSsiAbsEncoderParameterEx2 reads all parameters from EPOS4 SSI absolute encoder.

```c
BOOL VCS_GetSsiAbsEncoderParameterEx2(HANDLE KeyHandle, WORD NodeId, WORD* pDataRate, WORD* pNbOfSpecialDataBitsLeading, WORD* pNbOfMultiTurnDataBits, WORD* pNbOfMultiTurnPositionBits, WORD* pNbOfSingleTurnDataBits, WORD* pNbOfSingleTurnPositionBits, WORD* pNbOfSpecialDataBitsTrailing, BOOL* pInvertedPolarity, WORD* pTimeout, WORD* pPowerupTime, BOOL* pCheckFrame, BOOL* pReferenceReset, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pDataRate` | `WORD*` | Output | Parameter `pDataRate` |
| `pNbOfSpecialDataBitsLeading` | `WORD*` | Output | Parameter `pNbOfSpecialDataBitsLeading` |
| `pNbOfMultiTurnDataBits` | `WORD*` | Output | Parameter `pNbOfMultiTurnDataBits` |
| `pNbOfMultiTurnPositionBits` | `WORD*` | Output | Parameter `pNbOfMultiTurnPositionBits` |
| `pNbOfSingleTurnDataBits` | `WORD*` | Output | Parameter `pNbOfSingleTurnDataBits` |
| `pNbOfSingleTurnPositionBits` | `WORD*` | Output | Parameter `pNbOfSingleTurnPositionBits` |
| `pNbOfSpecialDataBitsTrailing` | `WORD*` | Output | Parameter `pNbOfSpecialDataBitsTrailing` |
| `pInvertedPolarity` | `BOOL*` | Output | Parameter `pInvertedPolarity` |
| `pTimeout` | `WORD*` | Output | Parameter `pTimeout` |
| `pPowerupTime` | `WORD*` | Output | Parameter `pPowerupTime` |
| `pCheckFrame` | `BOOL*` | Output | Parameter `pCheckFrame` |
| `pReferenceReset` | `BOOL*` | Output | Parameter `pReferenceReset` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetMaxFollowingError` (4.2.3.1)
**Description**: VCS_SetMaxFollowingError writes the maximal allowed following error parameter.

```c
BOOL VCS_SetMaxFollowingError(HANDLE KeyHandle, WORD NodeId, DWORD MaxFollowingError, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `MaxFollowingError` | `DWORD` | Input | Parameter `MaxFollowingError` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetMaxFollowingError` (4.2.3.2)
**Description**: VCS_GetMaxFollowingError reads the maximal allowed following error parameter.

```c
BOOL VCS_GetMaxFollowingError(HANDLE KeyHandle, WORD NodeId, DWORD* pMaxFollowingError, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pMaxFollowingError` | `DWORD*` | Output | Parameter `pMaxFollowingError` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetMaxProfileVelocity` (4.2.3.3)
**Description**: VCS_SetMaxProfileVelocity writes the maximal allowed velocity. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_SetMaxProfileVelocity(HANDLE KeyHandle, WORD NodeId, DWORD MaxProfileVelocity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `MaxProfileVelocity` | `DWORD` | Input | Parameter `MaxProfileVelocity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetMaxProfileVelocity` (4.2.3.4)
**Description**: VCS_GetMaxProfileVelocity reads the maximal allowed velocity. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_GetMaxProfileVelocity(HANDLE KeyHandle, WORD NodeId, DWORD* pMaxProfileVelocity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pMaxProfileVelocity` | `DWORD*` | Output | Parameter `pMaxProfileVelocity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetMaxAcceleration` (4.2.3.5)
**Description**: VCS_SetMaxAcceleration writes the maximal allowed acceleration/deceleration.

```c
BOOL VCS_SetMaxAcceleration(HANDLE KeyHandle, WORD NodeId, DWORD MaxAcceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `MaxAcceleration` | `DWORD` | Input | Parameter `MaxAcceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetMaxAcceleration` (4.2.3.6)
**Description**: VCS_GetMaxAcceleration reads the maximal allowed acceleration/deceleration.

```c
BOOL VCS_GetMaxAcceleration(HANDLE KeyHandle, WORD NodeId, DWORD* pMaxAcceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pMaxAcceleration` | `DWORD*` | Output | Parameter `pMaxAcceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetControllerGain` (4.2.4.1)
**Description**: VCS_SetControllerGain writes the controller gain.

```c
VCS_SetControllerGain(HANDLE KeyHandle, WORD NodeId, WORD EController, WORD EGain, DWORD64 Value, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `EController` | `WORD` | Input | Parameter `EController` |
| `EGain` | `WORD` | Input | Parameter `EGain` |
| `Value` | `DWORD64` | Input | Parameter `Value` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetControllerGain` (4.2.4.2)
**Description**: VCS_SetControllerGain reads the controller gain.

```c
VCS_GetControllerGain(HANDLE KeyHandle, WORD NodeId, WORD EController, WORD EGain, DWORD64* pValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `EController` | `WORD` | Input | Parameter `EController` |
| `EGain` | `WORD` | Input | Parameter `EGain` |
| `pValue` | `DWORD64*` | Output | Parameter `pValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DigitalInputConfiguration` (4.2.5.1)
**Description**: VCS_DigitalInputConfiguration sets the parameter for one digital input.

```c
BOOL VCS_DigitalInputConfiguration(HANDLE KeyHandle, WORD NodeId, WORD DigitalInputNb, WORD Configuration, BOOL Mask, BOOL Polarity, BOOL ExecutionMask, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DigitalInputNb` | `WORD` | Input | Parameter `DigitalInputNb` |
| `Configuration` | `WORD` | Input | Parameter `Configuration` |
| `Mask` | `BOOL` | Input | Parameter `Mask` |
| `Polarity` | `BOOL` | Input | Parameter `Polarity` |
| `ExecutionMask` | `BOOL` | Input | Parameter `ExecutionMask` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DigitalOutputConfiguration` (4.2.5.2)
**Description**: VCS_DigitalOutputConfiguration sets parameter for one digital output.

```c
BOOL VCS_DigitalOutputConfiguration(HANDLE KeyHandle, WORD NodeId, WORD DigitalOutputNb, WORD Configuration, BOOL State, BOOL Mask, BOOL Polarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DigitalOutputNb` | `WORD` | Input | Parameter `DigitalOutputNb` |
| `Configuration` | `WORD` | Input | Parameter `Configuration` |
| `State` | `BOOL` | Input | Parameter `State` |
| `Mask` | `BOOL` | Input | Parameter `Mask` |
| `Polarity` | `BOOL` | Input | Parameter `Polarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_AnalogInputConfiguration` (4.2.5.3)
**Description**: VCS_AnalogInputConfiguration sets the configuration parameter for one analog input.

```c
BOOL VCS_AnalogInputConfiguration(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNb, WORD Configuration, BOOL ExecutionMask, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNb` | `WORD` | Input | Parameter `AnalogInputNb` |
| `Configuration` | `WORD` | Input | Parameter `Configuration` |
| `ExecutionMask` | `BOOL` | Input | Parameter `ExecutionMask` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_AnalogOutputConfiguration` (4.2.5.4)
**Description**: VCS_AnalogOutputConfiguration sets the configuration parameter for one analog output.

```c
BOOL VCS_AnalogOutputConfiguration(HANDLE KeyHandle, WORD NodeId, WORD AnalogOutputNb, WORD Configuration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogOutputNb` | `WORD` | Input | Parameter `AnalogOutputNb` |
| `Configuration` | `WORD` | Input | Parameter `Configuration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetVelocityUnits` (4.2.6.1)
**Description**: VCS_SetVelocityUnits writes velocity unit parameters.

```c
BOOL VCS_SetVelocityUnits(HANDLE KeyHandle, WORD NodeId, BYTE VelDimension, char VelNotation, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `VelDimension` | `BYTE` | Input | Parameter `VelDimension` |
| `VelNotation` | `char` | Input | Parameter `VelNotation` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetVelocityUnits` (4.2.6.2)
**Description**: VCS_GetVelocityUnits reads velocity unit parameters.

```c
BOOL VCS_GetVelocityUnits(HANDLE KeyHandle, WORD NodeId, BYTE* pVelDimension, char* pVelNotation, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pVelDimension` | `BYTE*` | Output | Parameter `pVelDimension` |
| `pVelNotation` | `char*` | Output | Parameter `pVelNotation` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |


---

## Chapter 5: Operation Functions

### 5.1 Operation Mode Selection

#### `VCS_SetOperationMode` (5.1.1)
**Description**: VCS_SetOperationMode sets the operation mode. Modes marked with a triple asterisk (***) are automatically mapped to EPOS4-compatible firmware operation modes as to Table 5-20.

```c
BOOL VCS_SetOperationMode(HANDLE KeyHandle, WORD NodeId, __int8 Mode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Mode` | `__int8` | Input | Parameter `Mode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetOperationMode` (5.1.2)
**Description**: VCS_GetOperationMode returns the activated operation mode.

```c
BOOL VCS_GetOperationMode(HANDLE KeyHandle, WORD NodeId, __int8* pMode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pMode` | `__int8*` | Output | Parameter `pMode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.2 State Machine Control

#### `VCS_ResetDevice` (5.2.1)
**Description**: VCS_ResetDevice is used to send the NMT service "Reset Node". Command is without acknowledge.

```c
BOOL VCS_ResetDevice(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetState` (5.2.2)
**Description**: VCS_SetState writes the actual state machine state.

```c
BOOL VCS_SetState(HANDLE KeyHandle, WORD NodeId, WORD State, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `State` | `WORD` | Input | Parameter `State` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetEnableState` (5.2.3)
**Description**: VCS_SetEnableState changes the device state to "enable".

```c
BOOL VCS_SetEnableState(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetDisableState` (5.2.4)
**Description**: VCS_SetDisableState changes the device state to "disable".

```c
BOOL VCS_SetDisableState(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetQuickStopState` (5.2.5)
**Description**: VCS_SetQuickStopState changes the device state to "quick stop".

```c
BOOL VCS_SetQuickStopState(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ClearFault` (5.2.6)
**Description**: VCS_ClearFault changes the device state from "fault" to "disable".

```c
BOOL VCS_ClearFault(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetState` (5.2.7)
**Description**: VCS_GetState reads the new state of the state machine.

```c
BOOL VCS_GetState(HANDLE KeyHandle, WORD NodeId, WORD* pState, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pState` | `WORD*` | Output | Parameter `pState` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetEnableState` (5.2.8)
**Description**: VCS_GetEnableState checks if the device is enabled.

```c
BOOL VCS_GetEnableState(HANDLE KeyHandle, WORD NodeId, BOOL* pIsEnabled, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pIsEnabled` | `BOOL*` | Output | Parameter `pIsEnabled` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetDisableState` (5.2.9)
**Description**: VCS_GetDisableState checks if the device is disabled.

```c
BOOL VCS_GetDisableState(HANDLE KeyHandle, WORD NodeId, BOOL* pIsDisabled, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pIsDisabled` | `BOOL*` | Output | Parameter `pIsDisabled` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetQuickStopState` (5.2.10)
**Description**: VCS_GetQuickStopState returns the device state quick stop.

```c
BOOL VCS_GetQuickStopState(HANDLE KeyHandle, WORD NodeId, BOOL* pIsQuickStopped, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pIsQuickStopped` | `BOOL*` | Output | Parameter `pIsQuickStopped` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetFaultState` (5.2.11)
**Description**: VCS_GetFaultState returns the device state fault. Get error information if the device is in fault state ("Error Handling" on page 5-74).

```c
BOOL VCS_GetFaultState(HANDLE KeyHandle, WORD NodeId, BOOL* pIsInFault, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pIsInFault` | `BOOL*` | Output | Parameter `pIsInFault` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.3 Device Error Handling

#### `VCS_GetNbOfDeviceError` (5.3.1)
**Description**: VCS_GetNbOfDeviceError returns the number of actual errors that are recorded.

```c
BOOL VCS_GetNbOfDeviceError(HANDLE KeyHandle, WORD NodeId, BYTE* pNbDeviceError, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pNbDeviceError` | `BYTE*` | Output | Parameter `pNbDeviceError` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetDeviceErrorCode` (5.3.2)
**Description**: VCS_GetDeviceErrorCode returns the error code of the selected error number.

```c
BOOL VCS_GetDeviceErrorCode(HANDLE KeyHandle, WORD NodeId, BYTE ErrorNumber, DWORD* pDeviceErrorCode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ErrorNumber` | `BYTE` | Input | Parameter `ErrorNumber` |
| `pDeviceErrorCode` | `DWORD*` | Output | Parameter `pDeviceErrorCode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.4 Movement & Actual Value Sensor Acquisition

#### `VCS_GetMovementState` (5.4.1)
**Description**: VCS_GetMovementState checks if the drive has reached target.

```c
BOOL VCS_GetMovementState(HANDLE KeyHandle, WORD NodeId, BOOL* pTargetReached, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pTargetReached` | `BOOL*` | Output | Parameter `pTargetReached` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPositionIs` (5.4.2)
**Description**: VCS_GetPositionIs returns the position actual value.

```c
BOOL VCS_GetPositionIs(HANDLE KeyHandle, WORD NodeId, long* pPositionIs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pPositionIs` | `long*` | Output | Parameter `pPositionIs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetVelocityIs` (5.4.3)
**Description**: VCS_GetVelocityIs reads the velocity actual value. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_GetVelocityIs(HANDLE KeyHandle, WORD NodeId, long* pVelocityIs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pVelocityIs` | `long*` | Output | Parameter `pVelocityIs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetVelocityIsAveraged` (5.4.4)
**Description**: VCS_GetVelocityIsAveraged reads the velocity actual averaged value. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_GetVelocityIsAveraged(HANDLE KeyHandle, WORD NodeId, long* pVelocityIsAveraged, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pVelocityIsAveraged` | `long*` | Output | Parameter `pVelocityIsAveraged` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetCurrentIs` (5.4.5)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_GetCurrentIsEx instead.

**Description**: VCS_GetCurrentIs returns the current actual value.

```c
BOOL VCS_GetCurrentIs(HANDLE KeyHandle, WORD NodeId, short* pCurrentIs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCurrentIs` | `short*` | Output | Parameter `pCurrentIs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetCurrentIsEx` (5.4.6)
**Description**: VCS_GetCurrentIsEx returns the current actual value.

```c
BOOL VCS_GetCurrentIsEx(HANDLE KeyHandle, WORD NodeId, long* pCurrentIs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCurrentIs` | `long*` | Output | Parameter `pCurrentIs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetCurrentIsAveraged` (5.4.7)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_GetCurrentIsAveragedEx instead.

**Description**: VCS_GetCurrentIsAveraged returns the current actual averaged value.

```c
BOOL VCS_GetCurrentIsAveraged(HANDLE KeyHandle, WORD NodeId, short* pCurrentIsAveraged, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCurrentIsAveraged` | `short*` | Output | Parameter `pCurrentIsAveraged` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetCurrentIsAveragedEx` (5.4.8)
**Description**: VCS_GetCurrentIsAveragedEx returns the current actual averaged value.

```c
BOOL VCS_GetCurrentIsAveragedEx(HANDLE KeyHandle, WORD NodeId, long* pCurrentIsAveraged, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCurrentIsAveraged` | `long*` | Output | Parameter `pCurrentIsAveraged` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_WaitForTargetReached` (5.4.9)
**Description**: VCS_WaitForTargetReached waits until the state is changed to target reached or until the time is up.

```c
BOOL VCS_WaitForTargetReached(HANDLE KeyHandle, WORD NodeId, DWORD Timeout, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Timeout` | `DWORD` | Input | Parameter `Timeout` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.5 Profile Position Mode (PPM)

#### `VCS_ActivateProfilePositionMode` (5.5.1)
**Description**: VCS_ActivateProfilePositionMode changes the operational mode to "profile position mode".

```c
BOOL VCS_ActivateProfilePositionMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetPositionProfile` (5.5.2)
**Description**: VCS_SetPositionProfile sets the position profile parameters.

```c
BOOL VCS_SetPositionProfile(HANDLE KeyHandle, WORD NodeId, DWORD ProfileVelocity, DWORD ProfileAcceleration, DWORD ProfileDeceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ProfileVelocity` | `DWORD` | Input | Parameter `ProfileVelocity` |
| `ProfileAcceleration` | `DWORD` | Input | Parameter `ProfileAcceleration` |
| `ProfileDeceleration` | `DWORD` | Input | Parameter `ProfileDeceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPositionProfile` (5.5.3)
**Description**: VCS_GetPositionProfile returns the position profile parameters.

```c
BOOL VCS_GetPositionProfile(HANDLE KeyHandle, WORD NodeId, DWORD* pProfileVelocity, DWORD* pProfileAcceleration, DWORD* pProfileDeceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pProfileVelocity` | `DWORD*` | Output | Parameter `pProfileVelocity` |
| `pProfileAcceleration` | `DWORD*` | Output | Parameter `pProfileAcceleration` |
| `pProfileDeceleration` | `DWORD*` | Output | Parameter `pProfileDeceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_MoveToPosition` (5.5.4)
**Description**: VCS_MoveToPosition starts movement with position profile to target position.

```c
BOOL VCS_MoveToPosition(HANDLE KeyHandle, WORD NodeId, long TargetPosition, BOOL Absolute, BOOL Immediately, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `TargetPosition` | `long` | Input | Parameter `TargetPosition` |
| `Absolute` | `BOOL` | Input | Parameter `Absolute` |
| `Immediately` | `BOOL` | Input | Parameter `Immediately` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetTargetPosition` (5.5.5)
**Description**: VCS_GetTargetPosition returns the profile position mode target value.

```c
BOOL VCS_GetTargetPosition(HANDLE KeyHandle, WORD NodeId, long* pTargetPosition, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pTargetPosition` | `long*` | Output | Parameter `pTargetPosition` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_HaltPositionMovement` (5.5.6)
**Description**: VCS_HaltPositionMovement stops the movement with profile deceleration.

```c
BOOL VCS_HaltPositionMovement(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnablePositionWindow` (5.5.7.1)
**Description**: VCS_EnablePositionWindow activates the position window.

```c
BOOL VCS_EnablePositionWindow(HANDLE KeyHandle, WORD NodeId, DWORD PositionWindow, WORD PositionWindowTime, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `PositionWindow` | `DWORD` | Input | Parameter `PositionWindow` |
| `PositionWindowTime` | `WORD` | Input | Parameter `PositionWindowTime` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisablePositionWindow` (5.5.7.2)
**Description**: VCS_DisablePositionWindow deactivates the position window.

```c
BOOL VCS_DisablePositionWindow(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.6 Profile Velocity Mode (PVM)

#### `VCS_ActivateProfileVelocityMode` (5.6.1)
**Description**: VCS_ActivateProfileVelocityMode changes the operational mode to "profile velocity mode".

```c
BOOL VCS_ActivateProfileVelocityMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetVelocityProfile` (5.6.2)
**Description**: VCS_SetVelocityProfile sets the velocity profile parameters.

```c
BOOL VCS_SetVelocityProfile(HANDLE KeyHandle, WORD NodeId, DWORD ProfileAcceleration, DWORD ProfileDeceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ProfileAcceleration` | `DWORD` | Input | Parameter `ProfileAcceleration` |
| `ProfileDeceleration` | `DWORD` | Input | Parameter `ProfileDeceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetVelocityProfile` (5.6.3)
**Description**: VCS_GetVelocityProfile returns the velocity profile parameters.

```c
BOOL VCS_GetVelocityProfile(HANDLE KeyHandle, WORD NodeId, DWORD* pProfileAcceleration, DWORD* pProfileDeceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pProfileAcceleration` | `DWORD*` | Output | Parameter `pProfileAcceleration` |
| `pProfileDeceleration` | `DWORD*` | Output | Parameter `pProfileDeceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_MoveWithVelocity` (5.6.4)
**Description**: VCS_MoveWithVelocity starts the movement with velocity profile to target velocity. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_MoveWithVelocity(HANDLE KeyHandle, WORD NodeId, long TargetVelocity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `TargetVelocity` | `long` | Input | Parameter `TargetVelocity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetTargetVelocity` (5.6.5)
**Description**: VCS_GetTargetVelocity returns the profile velocity mode target value. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_GetTargetVelocity(HANDLE KeyHandle, WORD NodeId, long* pTargetVelocity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pTargetVelocity` | `long*` | Output | Parameter `pTargetVelocity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_HaltVelocityMovement` (5.6.6)
**Description**: VCS_HaltVelocityMovement stops the movement with profile deceleration.

```c
BOOL VCS_HaltVelocityMovement(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnableVelocityWindow` (5.6.7.1)
**Description**: VCS_EnableVelocityWindow activates the velocity window.

```c
BOOL VCS_EnableVelocityWindow(HANDLE KeyHandle, WORD NodeId, DWORD VelocityWindow, WORD VelocityWindowTime, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `VelocityWindow` | `DWORD` | Input | Parameter `VelocityWindow` |
| `VelocityWindowTime` | `WORD` | Input | Parameter `VelocityWindowTime` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisableVelocityWindow` (5.6.7.2)
**Description**: VCS_DisableVelocityWindow deactivates the velocity window.

```c
BOOL VCS_DisableVelocityWindow(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.7 Homing Mode (HM)

#### `VCS_ActivateHomingMode` (5.7.1)
**Description**: VCS_ActivateHomingMode changes the operational mode to "homing mode".

```c
BOOL VCS_ActivateHomingMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetHomingParameter` (5.7.2)
**Description**: VCS_SetHomingParameter writes all homing parameters. The parameter units depend on (position, velocity, acceleration) notation index.

```c
BOOL VCS_SetHomingParameter(HANDLE KeyHandle, WORD NodeId, DWORD HomingAcceleration, DWORD SpeedSwitch, DWORD SpeedIndex, long HomeOffset, WORD CurrentThreshold, long HomePosition, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `HomingAcceleration` | `DWORD` | Input | Parameter `HomingAcceleration` |
| `SpeedSwitch` | `DWORD` | Input | Parameter `SpeedSwitch` |
| `SpeedIndex` | `DWORD` | Input | Parameter `SpeedIndex` |
| `HomeOffset` | `long` | Input | Parameter `HomeOffset` |
| `CurrentThreshold` | `WORD` | Input | Parameter `CurrentThreshold` |
| `HomePosition` | `long` | Input | Parameter `HomePosition` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetHomingParameter` (5.7.3)
**Description**: VCS_GetHomingParameter reads all homing parameters. The parameter units depend on (position, velocity, acceleration) notation index.

```c
BOOL VCS_GetHomingParameter(HANDLE KeyHandle, WORD NodeId, DWORD* pHomingAcceleration, DWORD* pSpeedSwitch, DWORD* pSpeedIndex, long* pHomeOffset, WORD* pCurrentThreshold, long* pHomePosition, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pHomingAcceleration` | `DWORD*` | Output | Parameter `pHomingAcceleration` |
| `pSpeedSwitch` | `DWORD*` | Output | Parameter `pSpeedSwitch` |
| `pSpeedIndex` | `DWORD*` | Output | Parameter `pSpeedIndex` |
| `pHomeOffset` | `long*` | Output | Parameter `pHomeOffset` |
| `pCurrentThreshold` | `WORD*` | Output | Parameter `pCurrentThreshold` |
| `pHomePosition` | `long*` | Output | Parameter `pHomePosition` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_FindHome` (5.7.4)
**Description**: VCS_FindHome and HomingMethod permit to find the system home (for example, a home switch).

```c
BOOL VCS_FindHome(HANDLE KeyHandle, WORD NodeId, __int8 HomingMethod, DWORD* ErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `HomingMethod` | `__int8` | Input | Parameter `HomingMethod` |
| `ErrorCode` | `DWORD*` | Output | Parameter `ErrorCode` |

#### `VCS_StopHoming` (5.7.5)
**Description**: VCS_StopHoming interrupts homing.

```c
BOOL VCS_StopHoming(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DefinePosition` (5.7.6)
**Description**: VCS_DefinePosition uses homing method 35 (Actual Position) to set a new home position.

```c
BOOL VCS_DefinePosition(HANDLE KeyHandle, WORD NodeId, long HomePosition, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `HomePosition` | `long` | Input | Parameter `HomePosition` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetHomingState` (5.7.7)
**Description**: VCS_GetHomingState returns the states if the homing position is attained and if an homing error has occurred.

```c
BOOL VCS_GetHomingState(HANDLE KeyHandle, WORD NodeId, BOOL* pHomingAttained, BOOL* pHomingError, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pHomingAttained` | `BOOL*` | Output | Parameter `pHomingAttained` |
| `pHomingError` | `BOOL*` | Output | Parameter `pHomingError` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_WaitForHomingAttained` (5.7.8)
**Description**: VCS_WaitForHomingAttained waits until the homing mode is successfully terminated or until the time has elapsed.

```c
BOOL VCS_WaitForHomingAttained(HANDLE KeyHandle, WORD NodeId, DWORD Timeout, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Timeout` | `DWORD` | Input | Parameter `Timeout` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.8 Interpolated Position Mode (IPM)

#### `VCS_ActivateInterpolatedPositionMode` (5.8.1)
**Description**: VCS_ActivateInterpolatedPositionMode changes the operational mode to "interpolated position mode".

```c
BOOL VCS_ActivateInterpolatedPositionMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetIpmBufferParameter` (5.8.2)
**Description**: VCS_SetIpmBufferParameter sets warning borders of the data input.

```c
BOOL VCS_SetIpmBufferParameter(HANDLE KeyHandle, WORD NodeId, WORD UnderflowWarningLimit, WORD OverflowWarningLimit, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `UnderflowWarningLimit` | `WORD` | Input | Parameter `UnderflowWarningLimit` |
| `OverflowWarningLimit` | `WORD` | Input | Parameter `OverflowWarningLimit` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetIpmBufferParameter` (5.8.3)
**Description**: VCS_GetIpmBufferParameter reads warning borders and the max. buffer size of the data input.

```c
BOOL VCS_GetIpmBufferParameter(HANDLE KeyHandle, WORD NodeId, WORD* pUnderflowWarningLimit, WORD* pOverflowWarningLimit, DWORD* pMaxBufferSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pUnderflowWarningLimit` | `WORD*` | Output | Parameter `pUnderflowWarningLimit` |
| `pOverflowWarningLimit` | `WORD*` | Output | Parameter `pOverflowWarningLimit` |
| `pMaxBufferSize` | `DWORD*` | Output | Parameter `pMaxBufferSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ClearIpmBuffer` (5.8.4)
**Description**: VCS_ClearIpmBuffer clears the input buffer and enables access to the input buffer for drive functions.

```c
BOOL VCS_ClearIpmBuffer(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetFreeIpmBufferSize` (5.8.5)
**Description**: VCS_GetFreeIpmBufferSize reads the available buffer size.

```c
BOOL VCS_GetFreeIpmBufferSize(HANDLE KeyHandle, WORD NodeId, DWORD* pBufferSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pBufferSize` | `DWORD*` | Output | Parameter `pBufferSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_AddPvtValueToIpmBuffer` (5.8.6)
**Description**: VCS_AddPvtValueToIpmBuffer adds a new PVT reference point to the device.

```c
BOOL VCS_AddPvtValueToIpmBuffer(HANDLE KeyHandle, WORD NodeId, long Position, long Velocity, BYTE Time, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Position` | `long` | Input | Parameter `Position` |
| `Velocity` | `long` | Input | Parameter `Velocity` |
| `Time` | `BYTE` | Input | Parameter `Time` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_StartIpmTrajectory` (5.8.7)
**Description**: VCS_StartIpmTrajectory starts the IPM trajectory.

```c
BOOL VCS_StartIpmTrajectory(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_StopIpmTrajectory` (5.8.8)
**Description**: VCS_StopIpmTrajectory stops the IPM trajectory.

```c
BOOL VCS_StopIpmTrajectory(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetIpmStatus` (5.8.9)
**Description**: VCS_GetIpmStatus returns different warning and error states.

```c
BOOL VCS_GetIpmStatus(HANDLE KeyHandle, WORD NodeId, BOOL* pTrajectoryRunning, BOOL* pIsUnderflowWarning, BOOL* pIsOverflowWarning, BOOL* pIsVelocityWarning, BOOL* pIsAccelerationWarning, BOOL* pIsUnderflowError, BOOL* pIsOverflowError, BOOL* pIsVelocityError, BOOL* pIsAccelerationError, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pTrajectoryRunning` | `BOOL*` | Output | Parameter `pTrajectoryRunning` |
| `pIsUnderflowWarning` | `BOOL*` | Output | Parameter `pIsUnderflowWarning` |
| `pIsOverflowWarning` | `BOOL*` | Output | Parameter `pIsOverflowWarning` |
| `pIsVelocityWarning` | `BOOL*` | Output | Parameter `pIsVelocityWarning` |
| `pIsAccelerationWarning` | `BOOL*` | Output | Parameter `pIsAccelerationWarning` |
| `pIsUnderflowError` | `BOOL*` | Output | Parameter `pIsUnderflowError` |
| `pIsOverflowError` | `BOOL*` | Output | Parameter `pIsOverflowError` |
| `pIsVelocityError` | `BOOL*` | Output | Parameter `pIsVelocityError` |
| `pIsAccelerationError` | `BOOL*` | Output | Parameter `pIsAccelerationError` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.9 Position Mode (PM)

#### `VCS_ActivatePositionMode` (5.9.1)
**Description**: VCS_ActivatePositionMode changes the operational mode to "position mode".

```c
BOOL VCS_ActivatePositionMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetPositionMust` (5.9.2)
**Description**: VCS_SetPositionMust sets the position mode setting value.

```c
BOOL VCS_SetPositionMust(HANDLE KeyHandle, WORD NodeId, long PositionMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `PositionMust` | `long` | Input | Parameter `PositionMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPositionMust` (5.9.3)
**Description**: VCS_GetPositionMust reads the position mode setting value.

```c
BOOL VCS_GetPositionMust(HANDLE KeyHandle, WORD NodeId, long* pPositionMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pPositionMust` | `long*` | Output | Parameter `pPositionMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ActivateAnalogPositionSetpoint` (5.9.4.1)
**Description**: VCS_ActivateAnalogPositionSetpoint configures the selected analog input for analog position setpoint.

```c
BOOL VCS_ActivateAnalogPositionSetpoint(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNumber, float Scaling, long Offset, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNumber` | `WORD` | Input | Parameter `AnalogInputNumber` |
| `Scaling` | `float` | Input | Parameter `Scaling` |
| `Offset` | `long` | Input | Parameter `Offset` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DeactivateAnalogPositionSetpoint` (5.9.4.2)
**Description**: VCS_DeactivateAnalogPositionSetpoint disables the selected analog input for analog position setpoint.

```c
BOOL VCS_DeactivateAnalogPositionSetpoint(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNumber, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNumber` | `WORD` | Input | Parameter `AnalogInputNumber` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnableAnalogPositionSetpoint` (5.9.4.3)
**Description**: VCS_EnableAnalogPositionSetpoint enables the execution mask for analog position setpoint.

```c
BOOL VCS_EnableAnalogPositionSetpoint(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisableAnalogPositionSetpoint` (5.9.4.4)
**Description**: VCS_DisableAnalogPositionSetpoint disables the execution mask for analog position setpoint.

```c
BOOL VCS_DisableAnalogPositionSetpoint(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.10 Velocity Mode (VM)

#### `VCS_ActivateVelocityMode` (5.10.1)
**Description**: VCS_ActivateVelocityMode changes the operational mode to "velocity mode".

```c
BOOL VCS_ActivateVelocityMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetVelocityMust` (5.10.2)
**Description**: VCS_SetVelocityMust sets the velocity mode setting value. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_SetVelocityMust(HANDLE KeyHandle, WORD NodeId, long VelocityMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `VelocityMust` | `long` | Input | Parameter `VelocityMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetVelocityMust` (5.10.3)
**Description**: VCS_GetVelocityMust returns the velocity mode setting value. The velocity is interpreted according to the currently configured velocity unit.

```c
BOOL VCS_GetVelocityMust(HANDLE KeyHandle, WORD NodeId, long* pVelocityMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pVelocityMust` | `long*` | Output | Parameter `pVelocityMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ActivateAnalogVelocitySetpoint` (5.10.4.1)
**Description**: VCS_ActivateAnalogVelocitySetpoint configures the selected analog input for analog velocity setpoint.

```c
BOOL VCS_ActivateAnalogVelocitySetpoint(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNumber, float Scaling, long Offset, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNumber` | `WORD` | Input | Parameter `AnalogInputNumber` |
| `Scaling` | `float` | Input | Parameter `Scaling` |
| `Offset` | `long` | Input | Parameter `Offset` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DeactivateAnalogVelocitySetpoint` (5.10.4.2)
**Description**: VCS_DeactivateAnalogVelocitySetpoint disables the selected analog input for analog velocity setpoint.

```c
BOOL VCS_DeactivateAnalogVelocitySetpoint(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNumber, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNumber` | `WORD` | Input | Parameter `AnalogInputNumber` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnableAnalogVelocitySetpoint` (5.10.4.3)
**Description**: VCS_EnableAnalogVelocitySetpoint enables the execution mask for analog velocity setpoint.

```c
BOOL VCS_EnableAnalogVelocitySetpoint(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisableAnalogVelocitySetpoint` (5.10.4.4)
**Description**: VCS_DisableAnalogVelocitySetpoint disables the execution mask for analog velocity setpoint.

```c
BOOL VCS_DisableAnalogVelocitySetpoint(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.11 Current Mode (CM)

#### `VCS_ActivateCurrentMode` (5.11.1)
**Description**: VCS_ActivateCurrentMode changes the operational mode to "current mode".

```c
BOOL VCS_ActivateCurrentMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetCurrentMust` (5.11.2)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_GetCurrentMustEx instead.

**Description**: VCS_GetCurrentMust reads the current mode setting value.

```c
BOOL VCS_GetCurrentMust(HANDLE KeyHandle, WORD NodeId, short* pCurrentMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCurrentMust` | `short*` | Output | Parameter `pCurrentMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetCurrentMustEx` (5.11.3)
**Description**: VCS_GetCurrentMustEx reads the current mode setting value.

```c
BOOL VCS_GetCurrentMustEx(HANDLE KeyHandle, WORD NodeId, long* pCurrentMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCurrentMust` | `long*` | Output | Parameter `pCurrentMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetCurrentMust` (5.11.4)
> [!WARNING]
> The function is no longer recommended for implementation. Use VCS_SetCurrentMustEx instead.

**Description**: VCS_SetCurrentMust writes current mode setting value.

```c
BOOL VCS_SetCurrentMust(HANDLE KeyHandle, WORD NodeId, short CurrentMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `CurrentMust` | `short` | Input | Parameter `CurrentMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetCurrentMustEx` (5.11.5)
**Description**: VCS_SetCurrentMustEx writes current mode setting value.

```c
BOOL VCS_SetCurrentMustEx(HANDLE KeyHandle, WORD NodeId, long CurrentMust, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `CurrentMust` | `long` | Input | Parameter `CurrentMust` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ActivateAnalogCurrentSetpoint` (5.11.6.1)
**Description**: VCS_ActivateAnalogCurrentSetpoint configures the selected analog input for analog current setpoint.

```c
BOOL VCS_ActivateAnalogCurrentSetpoint(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNumber, float Scaling, short Offset, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNumber` | `WORD` | Input | Parameter `AnalogInputNumber` |
| `Scaling` | `float` | Input | Parameter `Scaling` |
| `Offset` | `short` | Input | Parameter `Offset` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DeactivateAnalogCurrentSetpoint` (5.11.6.2)
**Description**: VCS_DeactivateAnalogCurrentSetpoint disables the selected analog input for analog current setpoint.

```c
BOOL VCS_DeactivateAnalogCurrentSetpoint(HANDLE KeyHandle, WORD NodeId, WORD AnalogInputNumber, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `AnalogInputNumber` | `WORD` | Input | Parameter `AnalogInputNumber` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnableAnalogCurrentSetpoint` (5.11.6.3)
**Description**: VCS_EnableAnalogCurrentSetpoint enables the execution mask for analog current setpoint.

```c
BOOL VCS_EnableAnalogCurrentSetpoint(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisableAnalogCurrentSetpoint` (5.11.6.4)
**Description**: VCS_DisableAnalogCurrentSetpoint disables the execution mask for analog current setpoint.

```c
BOOL VCS_DisableAnalogCurrentSetpoint(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.12 Master Encoder Mode (MEM)

#### `VCS_ActivateMasterEncoderMode` (5.12.1)
**Description**: VCS_ActivateMasterEncoderMode changes the operational mode to "master encoder mode".

```c
BOOL VCS_ActivateMasterEncoderMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetMasterEncoderParameter` (5.12.2)
**Description**: VCS_SetMasterEncoderParameter writes all parameters for master encoder mode.

```c
BOOL VCS_SetMasterEncoderParameter(HANDLE KeyHandle, WORD NodeId, WORD ScalingNumerator, WORD ScalingDenominator, BYTE Polarity, DWORD MaxVelocity, DWORD MaxAcceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ScalingNumerator` | `WORD` | Input | Parameter `ScalingNumerator` |
| `ScalingDenominator` | `WORD` | Input | Parameter `ScalingDenominator` |
| `Polarity` | `BYTE` | Input | Parameter `Polarity` |
| `MaxVelocity` | `DWORD` | Input | Parameter `MaxVelocity` |
| `MaxAcceleration` | `DWORD` | Input | Parameter `MaxAcceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetMasterEncoderParameter` (5.12.3)
**Description**: VCS_GetMasterEncoderParameter reads all parameters for master encoder mode.

```c
BOOL VCS_GetMasterEncoderParameter(HANDLE KeyHandle, WORD NodeId, WORD* pScalingNumerator, WORD* pScalingDenominator, BYTE* pPolarity, DWORD* pMaxVelocity, DWORD* pMaxAcceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pScalingNumerator` | `WORD*` | Output | Parameter `pScalingNumerator` |
| `pScalingDenominator` | `WORD*` | Output | Parameter `pScalingDenominator` |
| `pPolarity` | `BYTE*` | Output | Parameter `pPolarity` |
| `pMaxVelocity` | `DWORD*` | Output | Parameter `pMaxVelocity` |
| `pMaxAcceleration` | `DWORD*` | Output | Parameter `pMaxAcceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.13 Step Direction Mode (SDM)

#### `VCS_ActivateStepDirectionMode` (5.13.1)
**Description**: VCS_ActivateStepDirectionMode changes the operational mode to "step direction mode".

```c
BOOL VCS_ActivateStepDirectionMode(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetStepDirectionParameter` (5.13.2)
**Description**: VCS_SetStepDirectionParameter writes all parameters for step direction mode.

```c
BOOL VCS_SetStepDirectionParameter(HANDLE KeyHandle, WORD NodeId, WORD ScalingNumerator, WORD ScalingDenominator, BYTE Polarity, DWORD MaxVelocity, DWORD MaxAcceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ScalingNumerator` | `WORD` | Input | Parameter `ScalingNumerator` |
| `ScalingDenominator` | `WORD` | Input | Parameter `ScalingDenominator` |
| `Polarity` | `BYTE` | Input | Parameter `Polarity` |
| `MaxVelocity` | `DWORD` | Input | Parameter `MaxVelocity` |
| `MaxAcceleration` | `DWORD` | Input | Parameter `MaxAcceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetStepDirectionParameter` (5.13.3)
**Description**: VCS_GetStepDirectionParameter reads all parameters for step direction mode.

```c
BOOL VCS_GetStepDirectionParameter(HANDLE KeyHandle, WORD NodeId, WORD* pScalingNumerator, WORD* pScalingDenominator, BYTE* pPolarity, DWORD* pMaxVelocity, DWORD* pMaxAcceleration, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pScalingNumerator` | `WORD*` | Output | Parameter `pScalingNumerator` |
| `pScalingDenominator` | `WORD*` | Output | Parameter `pScalingDenominator` |
| `pPolarity` | `BYTE*` | Output | Parameter `pPolarity` |
| `pMaxVelocity` | `DWORD*` | Output | Parameter `pMaxVelocity` |
| `pMaxAcceleration` | `DWORD*` | Output | Parameter `pMaxAcceleration` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 5.14 Digital & Analog Inputs/Outputs & Position Compare/Marker

#### `VCS_GetAllDigitalInputs` (5.14.1)
**Description**: VCS_GetAllDigitalInputs returns state of all digital inputs.

```c
BOOL VCS_GetAllDigitalInputs(HANDLE KeyHandle, WORD NodeId, WORD* pInputs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pInputs` | `WORD*` | Output | Parameter `pInputs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetAllDigitalOutputs` (5.14.2)
**Description**: VCS_GetAllDigitalOutputs returns state of all digital outputs.

```c
BOOL VCS_GetAllDigitalOutputs(HANDLE KeyHandle, WORD NodeId, WORD* pOutputs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pOutputs` | `WORD*` | Output | Parameter `pOutputs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetAllDigitalOutputs` (5.14.3)
**Description**: VCS_SetAllDigitalOutputs sets the state of all digital outputs.

```c
BOOL VCS_SetAllDigitalOutputs(HANDLE KeyHandle, WORD NodeId, WORD Outputs, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Outputs` | `WORD` | Input | Parameter `Outputs` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetAnalogInput` (5.14.4)
**Description**: VCS_GetAnalogInput returns the value from an analog input.

```c
BOOL VCS_GetAnalogInput(HANDLE KeyHandle, WORD NodeId, WORD InputNumber, WORD* pAnalogValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `InputNumber` | `WORD` | Input | Parameter `InputNumber` |
| `pAnalogValue` | `WORD*` | Output | Parameter `pAnalogValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetAnalogInputVoltage` (5.14.5)
**Description**: VCS_GetAnalogInputVoltage returns the voltage value from an analog input.

```c
BOOL VCS_GetAnalogInputVoltage(HANDLE KeyHandle, WORD NodeId, WORD InputNumber, long* pVoltageValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `InputNumber` | `WORD` | Input | Parameter `InputNumber` |
| `pVoltageValue` | `long*` | Output | Parameter `pVoltageValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetAnalogInputState` (5.14.6)
**Description**: VCS_GetAnalogInputState returns the state value from an analog input functionality.

```c
BOOL VCS_GetAnalogInputState(HANDLE KeyHandle, WORD NodeId, WORD Configuration, long* pStateValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Configuration` | `WORD` | Input | Parameter `Configuration` |
| `pStateValue` | `long*` | Output | Parameter `pStateValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetAnalogOutput` (5.14.7)
**Description**: VCS_SetAnalogOutput sets the voltage level of an analog output.

```c
BOOL VCS_SetAnalogOutput(HANDLE KeyHandle, WORD NodeId, WORD OutputNumber, WORD AnalogValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `OutputNumber` | `WORD` | Input | Parameter `OutputNumber` |
| `AnalogValue` | `WORD` | Input | Parameter `AnalogValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetAnalogOutputVoltage` (5.14.8)
**Description**: VCS_SetAnalogOutputVoltage sets the voltage level of an analog output.

```c
BOOL VCS_SetAnalogOutputVoltage(HANDLE KeyHandle, WORD NodeId, WORD OutputNumber, long VoltageValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `OutputNumber` | `WORD` | Input | Parameter `OutputNumber` |
| `VoltageValue` | `long` | Input | Parameter `VoltageValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetAnalogOutputState` (5.14.9)
**Description**: VCS_SetAnalogOutputState sets the state value for an analog output functionality.

```c
BOOL VCS_SetAnalogOutputState(HANDLE KeyHandle, WORD NodeId, WORD Configuration, long StateValue, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `Configuration` | `WORD` | Input | Parameter `Configuration` |
| `StateValue` | `long` | Input | Parameter `StateValue` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetPositionCompareParameter` (5.14.10.1)
**Description**: VCS_SetPositionCompareParameter writes all parameters for position compare.

```c
BOOL VCS_SetPositionCompareParameter(HANDLE KeyHandle, WORD NodeId, BYTE OperationalMode, BYTE IntervalMode, BYTE DirectionDependency, WORD IntervalWidth, WORD IntervalRepetitions, WORD PulseWidth, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `OperationalMode` | `BYTE` | Input | Parameter `OperationalMode` |
| `IntervalMode` | `BYTE` | Input | Parameter `IntervalMode` |
| `DirectionDependency` | `BYTE` | Input | Parameter `DirectionDependency` |
| `IntervalWidth` | `WORD` | Input | Parameter `IntervalWidth` |
| `IntervalRepetitions` | `WORD` | Input | Parameter `IntervalRepetitions` |
| `PulseWidth` | `WORD` | Input | Parameter `PulseWidth` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPositionCompareParameter` (5.14.10.2)
**Description**: VCS_GetPositionCompareParameter reads all parameters for position compare.

```c
BOOL VCS_GetPositionCompareParameter(HANDLE KeyHandle, WORD NodeId, BYTE* pOperationalMode, BYTE* pIntervalMode, BYTE* pDirectionDependency, WORD* pIntervalWidth, WORD* pIntervalRepetitions, WORD* pPulseWidth, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pOperationalMode` | `BYTE*` | Output | Parameter `pOperationalMode` |
| `pIntervalMode` | `BYTE*` | Output | Parameter `pIntervalMode` |
| `pDirectionDependency` | `BYTE*` | Output | Parameter `pDirectionDependency` |
| `pIntervalWidth` | `WORD*` | Output | Parameter `pIntervalWidth` |
| `pIntervalRepetitions` | `WORD*` | Output | Parameter `pIntervalRepetitions` |
| `pPulseWidth` | `WORD*` | Output | Parameter `pPulseWidth` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ActivatePositionCompare` (5.14.10.3)
**Description**: VCS_ActivatePositionCompare enables the output to position compare method.

```c
BOOL VCS_ActivatePositionCompare(HANDLE KeyHandle, WORD NodeId, WORD DigitalOutputNumber, BOOL Polarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DigitalOutputNumber` | `WORD` | Input | Parameter `DigitalOutputNumber` |
| `Polarity` | `BOOL` | Input | Parameter `Polarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DeactivatePositionCompare` (5.14.10.4)
**Description**: VCS_DeactivatePositionCompare disables the output to position compare method.

```c
BOOL VCS_DeactivatePositionCompare(HANDLE KeyHandle, WORD NodeId, WORD DigitalOutputNumber, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DigitalOutputNumber` | `WORD` | Input | Parameter `DigitalOutputNumber` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnablePositionCompare` (5.14.10.5)
**Description**: VCS_EnablePositionCompare enables the output mask for position compare method.

```c
BOOL VCS_EnablePositionCompare(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisablePositionCompare` (5.14.10.6)
**Description**: VCS_DisablePositionCompare disables the output mask from position compare method.

```c
BOOL VCS_DisablePositionCompare(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetPositionCompareReferencePosition` (5.14.10.7)
**Description**: VCS_SetPositionCompareReferencePosition writes the reference position for position compare method.

```c
BOOL VCS_SetPositionCompareReferencePosition(HANDLE KeyHandle, WORD NodeId, long ReferencePosition, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ReferencePosition` | `long` | Input | Parameter `ReferencePosition` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SetPositionMarkerParameter` (5.14.11.1)
**Description**: VCS_SetPositionMarkerParameter writes all parameters for position marker method.

```c
BOOL VCS_SetPositionMarkerParameter(HANDLE KeyHandle, WORD NodeId, BYTE PositionMarkerEdgeType, BYTE PositionMarkerMode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `PositionMarkerEdgeType` | `BYTE` | Input | Parameter `PositionMarkerEdgeType` |
| `PositionMarkerMode` | `BYTE` | Input | Parameter `PositionMarkerMode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetPositionMarkerParameter` (5.14.11.2)
**Description**: VCS_GetPositionMarkerParameter reads all parameters for position marker method.

```c
BOOL VCS_GetPositionMarkerParameter(HANDLE KeyHandle, WORD NodeId, BYTE* pPositionMarkerEdgeType, BYTE* pPositionMarkerMode, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pPositionMarkerEdgeType` | `BYTE*` | Output | Parameter `pPositionMarkerEdgeType` |
| `pPositionMarkerMode` | `BYTE*` | Output | Parameter `pPositionMarkerMode` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ActivatePositionMarker` (5.14.11.3)
**Description**: VCS_ActivatePositionMarker enables the digital input to position marker method.

```c
BOOL VCS_ActivatePositionMarker(HANDLE KeyHandle, WORD NodeId, WORD DigitalInputNumber, BOOL Polarity, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DigitalInputNumber` | `WORD` | Input | Parameter `DigitalInputNumber` |
| `Polarity` | `BOOL` | Input | Parameter `Polarity` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DeactivatePositionMarker` (5.14.11.4)
**Description**: VCS_DeactivatePositionMarker disables the digital input to position marker method.

```c
BOOL VCS_DeactivatePositionMarker(HANDLE KeyHandle, WORD NodeId, WORD DigitalInputNumber, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `DigitalInputNumber` | `WORD` | Input | Parameter `DigitalInputNumber` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ReadPositionMarkerCounter` (5.14.11.5)
**Description**: VCS_ReadPositionMarkerCounter returns the number of the detected edges.

```c
BOOL VCS_ReadPositionMarkerCounter(HANDLE KeyHandle, WORD NodeId, WORD* pCount, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pCount` | `WORD*` | Output | Parameter `pCount` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ReadPositionMarkerCapturedPosition` (5.14.11.6)
**Description**: VCS_ReadPositionMarkerCapturedPosition returns the captured position at the passed CounterIndex value.

```c
BOOL VCS_ReadPositionMarkerCapturedPosition(HANDLE KeyHandle, WORD NodeId, WORD CounterIndex, long* pCapturedPosition, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `CounterIndex` | `WORD` | Input | Parameter `CounterIndex` |
| `pCapturedPosition` | `long*` | Output | Parameter `pCapturedPosition` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ResetPositionMarkerCounter` (5.14.11.7)
**Description**: VCS_ResetPositionMarkerCounter clears the counter and the captured positions.

```c
BOOL VCS_ResetPositionMarkerCounter(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |


---

## Chapter 6: Data Recording Functions

### 6.1 Recorder Configuration & Trigger Setup

#### `VCS_SetRecorderParameter` (6.1.1)
**Description**: VCS_SetRecorderParameter writes parameters for data recorder.

```c
BOOL VCS_SetRecorderParameter(HANDLE KeyHandle, WORD NodeId, WORD SamplingPeriod, WORD NbOfPrecedingSamples, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `SamplingPeriod` | `WORD` | Input | Parameter `SamplingPeriod` |
| `NbOfPrecedingSamples` | `WORD` | Input | Parameter `NbOfPrecedingSamples` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_GetRecorderParameter` (6.1.2)
**Description**: VCS_GetRecorderParameter reads parameters for data recorder.

```c
BOOL VCS_GetRecorderParameter(HANDLE KeyHandle, WORD NodeId, WORD* pSamplingPeriod, WORD* pNbOfPrecedingSamples, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pSamplingPeriod` | `WORD*` | Output | Parameter `pSamplingPeriod` |
| `pNbOfPrecedingSamples` | `WORD*` | Output | Parameter `pNbOfPrecedingSamples` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_EnableTrigger` (6.1.3)
**Description**: VCS_EnableTrigger connects the trigger(s) for data recording.

```c
BOOL VCS_EnableTrigger(HANDLE KeyHandle, WORD NodeId, BYTE TriggerType, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `TriggerType` | `BYTE` | Input | Parameter `TriggerType` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DisableAllTriggers` (6.1.4)
**Description**: VCS_DisableAllTriggers sets data recorder configuration for triggers to zero.

```c
BOOL VCS_DisableAllTriggers(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ActivateChannel` (6.1.5)
**Description**: VCS_ActivateChannel connects object for data recording. Start with channel 1 (one)! Then, for every activated channel, the number of sampling variables will be incremented.

```c
BOOL VCS_ActivateChannel(HANDLE KeyHandle, WORD NodeId, BYTE ChannelNumber, WORD ObjectIndex, BYTE ObjectSubIndex, BYTE ObjectSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ChannelNumber` | `BYTE` | Input | Parameter `ChannelNumber` |
| `ObjectIndex` | `WORD` | Input | Parameter `ObjectIndex` |
| `ObjectSubIndex` | `BYTE` | Input | Parameter `ObjectSubIndex` |
| `ObjectSize` | `BYTE` | Input | Parameter `ObjectSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_DeactivateAllChannels` (6.1.6)
**Description**: VCS_DeactivateAllChannels zeros all data recording objects.

```c
BOOL VCS_DeactivateAllChannels(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 6.2 Recorder Execution & Status

#### `VCS_StartRecorder` (6.2.1)
**Description**: VCS_StartRecorder starts data recording.

```c
BOOL VCS_StartRecorder(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_StopRecorder` (6.2.2)
**Description**: VCS_StopRecorder stops data recording.

```c
BOOL VCS_StopRecorder(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ForceTrigger` (6.2.3)
**Description**: VCS_ForceTrigger forces the data recording triggers.

```c
BOOL VCS_ForceTrigger(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_IsRecorderRunning` (6.2.4)
**Description**: VCS_IsRecorderRunning returns the data recorder status "running".

```c
BOOL VCS_IsRecorderRunning(HANDLE KeyHandle, WORD NodeId, BOOL* pRunning, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pRunning` | `BOOL*` | Output | Parameter `pRunning` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_IsRecorderTriggered` (6.2.5)
**Description**: VCS_IsRecorderTriggered returns data recorder status "triggered".

```c
BOOL VCS_IsRecorderTriggered(HANDLE KeyHandle, WORD NodeId, BOOL* pTriggered, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pTriggered` | `BOOL*` | Output | Parameter `pTriggered` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 6.3 Data Extraction & File Export

#### `VCS_ReadChannelVectorSize` (6.3.1)
**Description**: VCS_ReadChannelVectorSize returns the maximal number of samples per variable. It is dynamically calculated by the data recorder.

```c
BOOL VCS_ReadChannelVectorSize(HANDLE KeyHandle, WORD NodeId, DWORD* pVectorSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pVectorSize` | `DWORD*` | Output | Parameter `pVectorSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ReadChannelDataVector` (6.3.2)
**Description**: VCS_ReadChannelDataVector returns the data points of a selected channel.

```c
BOOL VCS_ReadChannelDataVector(HANDLE KeyHandle, WORD NodeId, BYTE ChannelNumber, BYTE* pDataVectorBuffer, DWORD VectorBufferSize, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ChannelNumber` | `BYTE` | Input | Parameter `ChannelNumber` |
| `pDataVectorBuffer` | `BYTE*` | Output | Parameter `pDataVectorBuffer` |
| `VectorBufferSize` | `DWORD` | Input | Parameter `VectorBufferSize` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ShowChannelDataDlg` (6.3.3)
**Description**: VCS_ShowChannelDataDlg opens the dialog to show the data channel(s). Not available with Linux.

```c
BOOL VCS_ShowChannelDataDlg(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ExportChannelDataToFile` (6.3.4)
**Description**: VCS_ExportChannelDataToFile saves the data point in a file. Not available with Linux.

```c
BOOL VCS_ExportChannelDataToFile(HANDLE KeyHandle, WORD NodeId, char* FileName, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `FileName` | `char*` | Input | Parameter `FileName` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

### 6.4 Advanced Buffer Functions

#### `VCS_ReadDataBuffer` (6.4.1)
**Description**: VCS_ReadDataBuffer returns the buffer data points.

```c
BOOL VCS_ReadDataBuffer(HANDLE KeyHandle, WORD NodeId, BYTE* pDataBuffer, DWORD BufferSizeToRead, DWORD* pBufferSizeRead, WORD* pVectorStartOffset, WORD* pMaxNbOfSamples, WORD* pNbOfRecordedSamples, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `pDataBuffer` | `BYTE*` | Output | Parameter `pDataBuffer` |
| `BufferSizeToRead` | `DWORD` | Input | Parameter `BufferSizeToRead` |
| `pBufferSizeRead` | `DWORD*` | Output | Parameter `pBufferSizeRead` |
| `pVectorStartOffset` | `WORD*` | Output | Parameter `pVectorStartOffset` |
| `pMaxNbOfSamples` | `WORD*` | Output | Parameter `pMaxNbOfSamples` |
| `pNbOfRecordedSamples` | `WORD*` | Output | Parameter `pNbOfRecordedSamples` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ExtractChannelDataVector` (6.4.2)
**Description**: VCS_ExtractChannelDataVector returns the vector of a data channel.

```c
BOOL VCS_ExtractChannelDataVector(HANDLE KeyHandle, WORD NodeId, BYTE ChannelNumber, BYTE* pDataBuffer, DWORD BufferSize, BYTE* pDataVectorBuffer, DWORD VectorBufferSize, WORD VectorStartOffset, WORD MaxNbOfSamples, WORD NbOfRecordedSamples, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `ChannelNumber` | `BYTE` | Input | Parameter `ChannelNumber` |
| `pDataBuffer` | `BYTE*` | Output | Parameter `pDataBuffer` |
| `BufferSize` | `DWORD` | Input | Parameter `BufferSize` |
| `pDataVectorBuffer` | `BYTE*` | Output | Parameter `pDataVectorBuffer` |
| `VectorBufferSize` | `DWORD` | Input | Parameter `VectorBufferSize` |
| `VectorStartOffset` | `WORD` | Input | Parameter `VectorStartOffset` |
| `MaxNbOfSamples` | `WORD` | Input | Parameter `MaxNbOfSamples` |
| `NbOfRecordedSamples` | `WORD` | Input | Parameter `NbOfRecordedSamples` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |


---

## Chapter 7: Low-Layer CAN Communication Functions

### 7.1 Raw CAN Framing & NMT Services

#### `VCS_SendCANFrame` (7.1.1)
**Description**: VCS_SendCANFrame sends a general CAN frame to the CAN bus.

```c
BOOL VCS_SendCANFrame(HANDLE KeyHandle, WORD CobID, WORD Length, void* pData, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `CobID` | `WORD` | Input | Parameter `CobID` |
| `Length` | `WORD` | Input | Parameter `Length` |
| `pData` | `void*` | Output | Parameter `pData` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_ReadCANFrame` (7.1.2)
**Description**: VCS_ReadCANFrame reads a general CAN frame from the CAN bus.

```c
BOOL VCS_ReadCANFrame(HANDLE KeyHandle, WORD CobID, WORD Length, void* pData, DWORD Timeout, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `CobID` | `WORD` | Input | Parameter `CobID` |
| `Length` | `WORD` | Input | Parameter `Length` |
| `pData` | `void*` | Output | Parameter `pData` |
| `Timeout` | `DWORD` | Input | Parameter `Timeout` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_RequestCANFrame` (7.1.3)
**Description**: VCS_RequestCANFrame requests a general CAN frame from the CAN bus using Remote Transmit Request (RTR).

```c
BOOL VCS_RequestCANFrame(HANDLE KeyHandle, WORD CobID, WORD Length, void* pData, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `CobID` | `WORD` | Input | Parameter `CobID` |
| `Length` | `WORD` | Input | Parameter `Length` |
| `pData` | `void*` | Output | Parameter `pData` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

#### `VCS_SendNMTService` (7.1.4)
**Description**: VCS_SendNMTService is used to send a NMT protocol from a master to one slave/all slaves in a network. Command is without acknowledge.

```c
BOOL VCS_SendNMTService(HANDLE KeyHandle, WORD NodeId, WORD CommandSpecifier, DWORD* pErrorCode)
```

| Parameter | C Data Type | I/O | Description / Role |
| :--- | :--- | :--- | :--- |
| `(Return Value)` | `BOOL` | Output | `1` on success (`HANDLE` for OpenDevice), `0` on error |
| `KeyHandle` | `HANDLE` | Input | Device key handle |
| `NodeId` | `WORD` | Input | CANopen Node ID |
| `CommandSpecifier` | `WORD` | Input | Parameter `CommandSpecifier` |
| `pErrorCode` | `DWORD*` | Output | Pointer to error code buffer |

---

## 4. Error Codes & Diagnostics
When any function returns `0` (or `NULL` handle), `*pErrorCode` holds a 32-bit error value.
- `VCS_GetErrorInfo(DWORD ErrorCode, char* pErrorInfo, WORD MaxStrSize)` retrieves the human-readable string for the error code.
- Device errors can also be queried on the EPOS state machine via `VCS_GetDeviceErrorCode(HANDLE KeyHandle, WORD NodeId, BYTE DeviceErrorNumber, DWORD* pDeviceErrorCode, DWORD* pErrorCode)`.
