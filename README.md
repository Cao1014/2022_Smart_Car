# 2022 Smart Car
This repository includes the electrical designs of a smart car designed by in 2022.

(Note: _Computer Vision_ is not within the scope, sorry &#x1F915;)  
## Software:   
_Altium Designer_ 22.0.2 with student license  
_MDK-ARM 5.29.0.0_  default compiler version 5
## The whole design
Overall design of the car

<img src="https://user-images.githubusercontent.com/72595382/210047877-32ab4065-c544-45dd-a50a-52c7fa55614f.png" width="380" height="360"> &nbsp; &nbsp; <img src="https://user-images.githubusercontent.com/72595382/210048130-d53eab96-e343-43d6-b916-31ef80964160.png" width="360" height="360">

## Brushed DC Motor Control

#### Circuit Description

Brushed DC motor are commonly used in industrial applications such as robots, valves and healthcare equipment. When only one direction of rotation is required, a single switch topology with PWM modulation can be used to vary the voltage applied to the motor, and thus to control its speed. When positioning is required or when both directions of rotation are needed a full H-bridge with PWM control is used.  
This design adopts the _H-bridge_ structure to drive the brushed DC motor.
![绘图2](https://user-images.githubusercontent.com/72595382/210053351-1f2f0f1a-1ded-45ea-87a8-30c49fce8814.svg)


**The circuit schematics and PCB layout are in the Altium folder.**  
**ESC codes are in the Codes folder.**
## Task Implementation

Obtaining the speed and angle of the car is the premise to realize the car's autonomous line tracking. The collected data form sensors are used in conjunction with the PID algorithm based on STM32. Through calculation, the control data via the PWM will be sent to the motor electronic speed controller to control speed of the motors. The whole mechanism is shown below

<img src="https://user-images.githubusercontent.com/72595382/210049955-0ab55a18-9b76-4acc-8020-871a66178923.png" width="640" height="260">  

![绘图1](https://user-images.githubusercontent.com/72595382/210053551-74426c39-0b63-439a-b6be-74f061800b84.svg)

**The circuit schematics and PCB layout are in the Altium folder.**  
**Codes for all patios are in the Codes folder.**  


