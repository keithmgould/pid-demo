// Includes for OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// for OLED: make sure the header file is correct
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#define ENCODER_1_PIN 7         // interrupt-0
#define ENCODER_2_PIN 14         // gpio
#define MOTOR_DIRECTION_PIN 15  // gpio
#define MOTOR_SPEED_PIN 9       // PWM

#define POT_P_PIN 0 // analogue pin 0
#define POT_I_PIN 1 // analogue pin 1
#define POT_D_PIN 2 // analogue pin 2

Adafruit_SSD1306 oled(4); // instantiate our OLED display

int kP = 0;       // holds the P parameter
int kI = 0;       // holds the I parameter
int kD = 0;       // holds the D parameter

int motorCommand = 0; // holds the final command to motor

int tick=0;  // holds the encoder's position (this is our error)

float accumulatedError = 0; // holds accumulated error

int loopTimeMs = 20; // time in milliseconds. ex: 20 yields 50Hz


void encoderEvent() {
  if(digitalRead(ENCODER_1_PIN) == digitalRead(ENCODER_2_PIN)){
    tick++;
  } else {
    tick--;
  }
}

void setup(){
  // Open console serial communications
  // initialize with the I2C addr 0x3C
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // init interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), encoderEvent, CHANGE);
}

// arduino analogRead range is 0-1023.
// lets convert to 0-99.
int normalize(int potVal){
  return int(potVal * 0.09775);
}

int fetchAnalog(int potPin){
  int potVal = analogRead(potPin);
  return normalize(potVal);
}

void updatePIDValues(){
  kP = int(fetchAnalog(POT_P_PIN) / 10);
  kI = fetchAnalog(POT_I_PIN);
  kD = fetchAnalog(POT_D_PIN);
}

int generatePCommand(int Kp, int error){
  return Kp * error;
}

int generateICommand(int Ki, int error){
  return Ki * accumulatedError;
}

int generateDCommand(int Kd, int error){
  return 0;
}

int generateMotorCommand(int Kp, int Ki, int Kd, int error){
  return generatePCommand(Kp, error) +
         generateICommand(Ki, error) +
         generateDCommand(Kd, error);
}

void updateMotor(int motorCommand){

  // safety first
  motorCommand = constrain(motorCommand, -255, 255);

  if(motorCommand >= 0){
    digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
  }else{
    digitalWrite(MOTOR_DIRECTION_PIN, LOW);
  }

  analogWrite(MOTOR_SPEED_PIN, abs(motorCommand));
}

void updateDisplay(){
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.print("P: ");
  oled.print(kP);
  oled.print(", I: ");
  oled.print(kI);
  oled.print(", D: ");
  oled.println(kD);
  oled.print("\nerror: ");
  oled.println(tick);
  oled.print("\naccError: ");
  oled.println(accumulatedError);
  oled.print("\ncommand: ");
  oled.println(motorCommand);
  oled.display();
}

void loop(){
  updatePIDValues();
  accumulatedError += tick;
  motorCommand = generateMotorCommand(kP, kI, kD, tick);
  updateMotor(motorCommand);
  updateDisplay();
  delay(20); // stop the program for some time
 }

