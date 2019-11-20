#include <AccelStepper.h>
#include <MultiStepper.h>

#define LEDpin 6
#define buttonPin 2

// Define number of steps per rotation:
const int stepsPerRevolution = 4076;

// Direction of revolutions
int dir = 1;

// Wiring:
// Pin 8 to IN1 on the ULN2003 driver
// Pin 9 to IN2 on the ULN2003 driver
// Pin 10 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver
#define IN1  8
#define IN2  9
#define IN3  10
#define IN4  11

AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

void takePhoto(void) {
  int i;
  for (i = 0; i < 76; i++) {
    digitalWrite(LEDpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(LEDpin, LOW);
    delayMicroseconds(7);
  }
  delay(27);
  delayMicroseconds(810);
  for (i = 0; i < 16; i++) {
    digitalWrite(LEDpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(LEDpin, LOW);
    delayMicroseconds(7);
  }
  delayMicroseconds(1540);
  for (i = 0; i < 16; i++) {
    digitalWrite(LEDpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(LEDpin, LOW);
    delayMicroseconds(7);
  }
  delayMicroseconds(3545);
  for (i = 0; i < 16; i++) {
    digitalWrite(LEDpin, HIGH);
    delayMicroseconds(7);
    digitalWrite(LEDpin, LOW);
    delayMicroseconds(7);
  }
}

void setup() {
  pinMode(buttonPin, INPUT);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(4000);
  stepper.setSpeed(1000);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), takePhoto, RISING);
  Serial.begin(9600);
}
void loop() {
  
  
  if (digitalRead(buttonPin) == 1){
    takePhoto();
    if (stepper.distanceToGo() == 0){
      stepper.move(stepsPerRevolution*2);
  }
  }
  //Serial.println(stepper.distanceToGo());
  stepper.run();
  
}
