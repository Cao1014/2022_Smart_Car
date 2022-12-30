# 2022 TDPS
This repository includes the electrical designs of TDPS, Glasgow College, UESTC, 2022 by Group01_Team03.
## The whole design
Overall design of the car

<img src="https://user-images.githubusercontent.com/72595382/210047877-32ab4065-c544-45dd-a50a-52c7fa55614f.png" width="380" height="360"> &nbsp; &nbsp; <img src="https://user-images.githubusercontent.com/72595382/210048130-d53eab96-e343-43d6-b916-31ef80964160.png" width="360" height="360">

## Brushed DC Motor Control

#### Circuit Description

Brushed DC motor are commonly used in industrial applications such as robots, valves and healthcare equipment. When only one direction of rotation is required, a single switch topology with PWM modulation can be used to vary the voltage applied to the motor, and thus to control its speed. When positioning is required or when both directions of rotation are needed a full H-bridge with PWM control is used.
This design adopts the _H-bridge_ structure to drive the brushed DC motor.

**The circuit schematics and PCB layout are in the Altium folder.**  
**ESC codes are in the Codes folder.**
## Task Implementation

Obtaining the speed and angle of the car is the premise to realize the car's autonomous line tracking. The collected data form sensors are used in conjunction with the PID algorithm based on STM32. Through calculation, the control data via the PWM will be sent to the motor electronic speed controller to control speed of the motors. The whole mechanism is shown below

<img src="https://user-images.githubusercontent.com/72595382/210049955-0ab55a18-9b76-4acc-8020-871a66178923.png" width="640" height="260">

**The circuit schematics and PCB layout are in the Altium folder.**  
**Codes for all patios are in the Codes folder.**  
(Note: Though the name is CHAS_G0_V2_Patio1, they include both Patio1 and Patio2 &#x1F600;)
