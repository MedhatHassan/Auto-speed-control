# Automatica speed control car with PID

## Requirments
- Nano ardino
- H-bridge LM
- IR photoelectric encoder sensor
- 2x16 LCD with I2c
- Car kit
- small grid board
- 4 dc motors
- 3x3.7v battary
  
## Project Description:
The Arduino code is designed to implement an automatic speed control system for a car using a Proportional-Integral-Derivative (PID) controller. The PID controller helps maintain a constant speed based on the desired RPM setpoint.

The code uses an IR photoelectric encoder sensor to detect the rotations per minute (RPM) of the car's wheels. The PID algorithm calculates the appropriate motor control output to adjust the car's speed and maintain the desired setpoint.

Conclusion:
This code provides a foundation for building an automatic speed control system using PID for a car. Adjustments to PID parameters may be necessary based on the specific characteristics of the car and the desired performance.
