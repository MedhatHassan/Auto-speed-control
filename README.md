# Automatica speed control car with PID

## Requirments
- Nano ardino
- H-bridge L298
- IR photoelectric encoder sensor
- 2x16 LCD with I2c
- Car kit
- small grid board
- 4 dc motors
- 3x3.7v battary
  
## Project Description:
The Arduino code is designed to implement an automatic speed control system for a car using a Proportional-Integral-Derivative (PID) controller. The PID controller helps maintain a constant speed based on the desired RPM setpoint.
The code uses an IR photoelectric encoder sensor to detect the rotations per minute (RPM) of the car's wheels. The PID algorithm calculates the appropriate motor control output to adjust the car's speed and maintain the desired setpoint.

# Performance:
1. error with H-bridge and PWM control to obtain 240 RPM was more than **20%**
   ![with out PID](performance%20graphs/Speed%20values%20without%20PID.png)
2. error after using try and error method to design PID values **8%**
    ![with PID1](performance%20graphs/Speed%20control%20with%20PID%20(Kp%3D2%20%2C%20Ki%3D0.1%2C%20Kd%3D0.2).png)

# Connections:
Connect the components to the **Nano Arduino** following these instructions:

### IR Photoelectric Encoder Sensor:
Connect the digital output of the IR sensor to D2 on the Nano Arduino.

### LCD (2x16 with I2C):
Connect the SDA pin of the LCD to A4 on the Nano Arduino.
Connect the SCL pin of the LCD to A5 on the Nano Arduino.

### H-Bridge (L298) Motor Control:
Motor A:
Connect ENA (PWM) to D10 on the Nano Arduino.
Connect IN1 to D6 and IN2 to D4 on the Nano Arduino.
Motor B:
Connect ENB (PWM) to D9 on the Nano Arduino.
Connect IN3 to D5 and IN4 to D3 on the Nano Arduino.

### RPM calculation:
Connect the digital output of the IR sensor to D2 on the Nano Arduino.
Attach an interrupt (INT0) to D2 to count the holes.

### Power Supply:
Connect the 3x3.7V batteries (+) to the 12v input in the H-bridge to power the motors and the (-) to the GND.
Connect the Nano Arduino to the 5v output & the GND of the H-bridge.

## Functions:

### define variables:
LCD (I2C address: 0x27, 16x2): Connected to pins SDA and SCL
IR Sensor Digital Output: D2
Interrupt for RPM Counting: D2
PID Controller Parameters:
Proportional Gain (Kp): 0.036
Integral Gain (Ki): 0.1
Derivative Gain (Kd): 0.2
Setpoint: 240 RPM

### setup():
Initializes the LCD and sets up pins and interrupts.
Initializes input and output pins.

### loop():
Calls CALC_RPM to calculate RPM.
Implements the PID control algorithm.
Calls moveMotors to adjust motor speed based on PID output.

### print_to_LCD():
Prints setpoint and actual RPM on the LCD.
moveMotors(double output):

Controls the motors based on the PID output.
### CALC_RPM():

Calculates RPM based on the holes detected by the IR sensor.
Updates LCD with setpoint and actual RPM.
### count():
Interrupt service routine to count holes detected by the IR sensor.

## Operation:
The IR sensor detects rotations of the car's wheels.
The PID algorithm calculates the appropriate motor control output based on the difference between the setpoint and actual RPM.
The motors' speed is adjusted accordingly to maintain a constant speed.

## Conclusion:
This code provides a foundation for building an automatic speed control system using PID for a car. Adjustments to PID parameters may be necessary based on the Transfer function of the car and the desired performance.
