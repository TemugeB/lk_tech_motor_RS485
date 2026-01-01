# LK-TECH MG8008 Motor Driver (RS485/Python)

A lightweight, high-performance Python driver for the **LK-TECH MG8008 series** (and similar MF/MS) BLDC actuators.

## Key Features

* **Closed-Loop Control:** Support for Speed and Multi-turn Position closed-loop control modes.
* **Absolute Position Tracking:** Uses 64-bit multi-loop commands to maintain absolute output shaft position across multiple rotations.
* **High-Frequency Feedback:** Efficiently parses Temperature, Torque Current (IQ), and Velocity from driver responses.

## Hardware Setup

* **Baudrate:** 115200 bps (Default).
* **Protocol:** RS485 (Half-duplex).
* **ID:** Default Motor ID is `1`.
* **Reduction Ratio:** ``9``. This is the ``i`` number of the motor. For example, ``MG8008E-i9`` <- this last number. 

## Quick Start

### Installation
Ensure you have `pyserial` installed:
```bash
pip install pyserial
```

### Test
Run the driver as a stand-alone program.
```bash
python3 lkm_driver.py
```

## Troubleshooting

### Connection Issues

Ensure you can see the device.

```bash
ls /dev/ttyUSB0
```

Allow user to dialout via the serial device.

```bash
sudo chmod a+rw /dev/ttyUSB0
```

### Driver Exceptions

``Motor responded but response length too short. Check connection with motor.`` - 
This means the motor responded but the driver could not parse message. Try changing baudrate and/or the motor_id. 

``Response checksum failed. Likely poor motor connection`` - 
Likely the signal degraded. Ensure you have the terminal resistors connected. On the motor side, switch #4 needs to be turned on. You still should connect the terminal resistors on the adapter side. 

### Other Tips

You should connect the termination resistors to ensure signal integrity. On the MG8008 motor, the termination resistor is engaged by flipping the 4th switch on the back. I also connect an additional resistor on the RS485 adapter side. The ``docs`` folder contain relevant data for the MG8008 motor. But I think this driver will work with other similar LK-Tech motors with minimal modification. 