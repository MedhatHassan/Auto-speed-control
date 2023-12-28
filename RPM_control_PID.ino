#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ENA 10   // PWM pin for motor A speed control
#define IN1 6    // Motor A input 1 - Change this to another pin if needed
#define IN2 4    // Motor A input 2

#define ENB 9    // PWM pin for motor B speed control
#define IN3 5    // Motor B input 1
#define IN4 3    // Motor B input 2

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
int DIGITAL_OUT = 2; // Pin D0 for digital ouput of IR sensor

//RPM calculation variables
float RPM = 0, laps = 0, comp = 0;
volatile int holes = 0;

//PID controller values
double Kp = 0.036; // Proportional gain
double Ki = 0.1; // Integral gain
double Kd = 0.2; // Derivative gain

double previousError = 0;
double integral = 0;
const int setpoint = 240; // Desired RPM setpoint

void setup() {
  //Inicialte the LCD 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Speed Sensor");
  lcd.setCursor(0, 1);
  lcd.print("PID Test");

  pinMode(DIGITAL_OUT, INPUT);  // Make the digital pin for IR sensor input
  attachInterrupt(digitalPinToInterrupt(DIGITAL_OUT), count, RISING);

  //Set mode for H-bridge pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  delay(1000);
  lcd.clear();
}

void loop() {
  CALC_RPM();

  double error = setpoint - RPM;
  integral += error;
  double derivative = error - previousError;

  // Calculate PID output
  double output = Kp * error + Ki * integral + Kd * derivative;

  // Apply PID output to motor control
  moveMotors(output);

  // Update variables for the next iteration
  previousError = error;

  delay(1000);
}

void print_to_LCD() {
  lcd.setCursor(0, 0);
  lcd.print("Setpoint : ");
  lcd.print(setpoint);
  lcd.print("    ");
  lcd.setCursor(0, 1);
  lcd.print("Actual RPM : ");
  lcd.print(RPM);
  lcd.print("    ");
}

void moveMotors(double output){
  // Motor control logic goes here based on PID output
  if (output > 0) {
    // Right Motors
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Lift Motors
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
    analogWrite(ENA, output); // Adjust the PWM value for speed control
  } else {
    // Right Motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    // Lift Motors
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
    analogWrite(ENA, output); // Adjust the PWM value for speed control
  }
}

void CALC_RPM(){
  detachInterrupt(digitalPinToInterrupt(DIGITAL_OUT));
  laps = (float(holes) / 20);
  RPM = laps * 60;
  // accumulate the distance traveled based on the detected laps.
  // 2 * π * radius (radius 6)
  comp = comp + PI * 6 * laps;
  print_to_LCD();
  holes = 0;
  attachInterrupt(digitalPinToInterrupt(DIGITAL_OUT), count, RISING);
}
void count() {
  holes++;
}
