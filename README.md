# Dodge Patrol VCM

Project Overview

The Vehicle Control Module (VCM) is an ESP32-S3 based controller that completely replaces the original Kids Trax electronics while retaining the original pedal, forward/reverse shifter, steering wheel, lights, and motors.

The VCM accepts control commands from four independent sources:

* Original steering wheel and pedal
* HOTRC DS-650 radio receiver
* Wi-Fi dashboard (iPhone)
* Future autonomous functions

The ESP32 continuously determines which control source has priority and commands the motors and steering accordingly.

⸻

Major Features

Drive-by-wire steering

The steering wheel is mechanically disconnected from the steering rack.

Instead:

Steering Wheel
↓
Hall Angle Sensor (SPI)
↓
ESP32
↓
PID Controller
↓
BTS7960
↓
Firgelli Actuator
↓
Steering Linkage

Advantages

* Adjustable steering ratio
* Steering sensitivity
* Parent override
* Automatic return-to-center
* Software calibration
* No steering wear

⸻

Closed Loop Steering

The Firgelli actuator contains a 10k feedback potentiometer.

The ESP32 constantly compares

Requested steering angle

vs

Actual steering angle

and adjusts motor power using PID control.

Dashboard displays

* Requested angle
* Actual angle
* Error
* Steering current

⸻

Independent Rear Motor Control

Each rear gearbox receives its own BTS7960.

ESP32
↓
Left BTS7960
↓
Left Motor
ESP32
↓
Right BTS7960
↓
Right Motor

Allows

* Smooth acceleration
* Regenerative braking simulation
* Electronic differential
* Traction control
* Wheel slip detection

⸻

Wheel Speed Sensors

Each gearbox

* 6 magnets
* 2 Hall sensors

Produces

* Left RPM
* Right RPM
* Direction
* Distance
* Odometer
* Slip

⸻

Steering Sensor

Hall Effect SPI encoder

Provides

4096 positions

Approximately

0.088°

resolution

⸻

Current Monitoring

INA3221

Channel 1

Left Motor

Channel 2

Right Motor

Channel 3

Steering Actuator

Dashboard

Battery Voltage

Motor Current

Motor Power

Steering Current

Peak Current

Average Current

⸻

Vehicle Motion

MPU6050

Measures

Pitch

Roll

Acceleration

Braking

Impact Detection

Yaw Rate

⸻

Remote Control

HOTRC Receiver

Provides

Throttle

Steering

Aux1

Aux2

Aux3

Aux4

These channels become software configurable.

⸻

iPhone Dashboard

The ESP32 hosts its own Wi-Fi web server.

Pages

Home

Vehicle Dashboard

Diagnostics

Configuration

Calibration

Fault Log

Firmware Update

⸻

Dashboard

Displays

Battery Voltage

Motor Current

Wheel Speed

Distance

Trip

Steering

Throttle

RC Signal

Wi-Fi Signal

Temperature

CPU Usage

Memory Usage

Loop Rate

⸻

Diagnostics

Graphs

Battery Voltage

Motor Current

Left Motor

Right Motor

Steering Current

Speed

Acceleration

Pitch

Roll

Steering Error

⸻

Configuration

Acceleration Ramp

Braking Ramp

Maximum Speed

Reverse Speed

Steering Sensitivity

Steering Deadband

PID Gains

Wheel Diameter

Magnets Per Wheel

RC Timeout

Parent Override

Throttle Curve

Battery Low Voltage

⸻

Calibration

Steering Center

Steering Limits

Actuator Limits

Wheel Diameter

Current Sensor Offset

Hall Sensor Direction

Motor Direction

Encoder Direction

⸻

Fault Detection

RC Lost

Wi-Fi Lost

Low Battery

Motor Stall

Steering Stall

Motor Overcurrent

Actuator Fault

Wheel Slip

Sensor Failure

⸻

Suggested PCB

I would use automotive screw terminals.

Power

Battery

Battery+

Battery-

Left Motor

M+

M-

Right Motor

M+

M-

Steering Actuator

Motor+

Motor-

Feedback+

Feedback

Feedback-

Lights

Siren

Accessory

⸻

ESP32 Layout

I would dedicate pins rather than sharing functionality.

I²C Bus

GPIO8 — SDA

GPIO9 — SCL

Connected to

* INA3221
* MPU6050
* Future display
* Future sensors

⸻

SPI Steering Encoder

GPIO10 — SCLK

GPIO11 — MISO

GPIO12 — MOSI

GPIO13 — CS

⸻

Left Motor

GPIO14 — RPWM

GPIO15 — LPWM

⸻

Right Motor

GPIO16 — RPWM

GPIO17 — LPWM

⸻

Steering BTS7960

GPIO18 — RPWM

GPIO21 — LPWM

⸻

Wheel Sensors

Left Encoder A

GPIO4

Left Encoder B

GPIO5

Right Encoder A

GPIO6

Right Encoder B

GPIO7

These should all be interrupt-capable inputs.

⸻

Firgelli Feedback Potentiometer

GPIO1 (ADC)

⸻

Battery Voltage Divider

GPIO2 (ADC)

⸻

Motor Wire Sense (Original Pedal/Shifter)

Left Motor Wire A

GPIO3 (ADC)

Left Motor Wire B

GPIO46 (ADC or digital depending on final ESP32-S3 module capabilities)

These two inputs let the ESP32 determine:

* Neutral
* Forward
* Reverse

using the original wiring.

⸻

HOTRC Receiver

Rather than dedicating six PWM input pins, I strongly recommend converting the receiver to a serial protocol if the receiver supports it (SBUS, iBUS, etc.). If it only outputs PWM, use:

GPIO35 — Ch1 Steering

GPIO36 — Ch2 Throttle

GPIO37 — Ch3

GPIO38 — Ch4

GPIO39 — Ch5

GPIO40 — Ch6

This keeps all receiver inputs grouped together.

⸻

Lights

GPIO41

MOSFET

⸻

Siren

GPIO42

MOSFET

⸻

Status RGB LED

GPIO45

WS2812

⸻

User Button

GPIO0

⸻

PCB Features I’d Include

* Automotive blade fuse for battery input
* Reverse-polarity protection (ideal diode or P-channel MOSFET)
* Main power switch input
* TVS diode on the 12 V input
* Separate 5 V and 3.3 V regulators
* Individual connector labels silkscreened on the PCB
* Test pads for all major signals
* USB-C for programming
* USB ESD protection
* Reset and Boot buttons
* Mounting holes sized for vibration-resistant standoffs

⸻

Firmware Architecture

Instead of writing one large Arduino sketch, organize it into independent tasks:

* Vehicle Control Manager
* Steering PID Controller
* Motor Control Manager
* Wheel Encoder Manager
* Current Monitor Manager
* IMU Manager
* RC Receiver Manager
* Web Server Manager
* Configuration Storage Manager
* Diagnostics Logger
* OTA Firmware Update Manager
* Safety Manager (highest priority)

Using FreeRTOS tasks on the ESP32-S3 will keep timing predictable and make future expansion much easier.

One hardware addition I’d strongly recommend

The only significant component I think is still missing is a CAN bus transceiver (for example, an MCP2562FD or SN65HVD230 if you choose classic CAN). Even if you don’t use it today, adding footprints for a CAN controller/transceiver and a connector gives you an expansion path for future accessories or additional control modules without redesigning the PCB. It costs very little in board space now but provides a professional-grade communication backbone if the project grows.