#include <TimerOne.h>
#include <AccelStepper.h>

// Motor Wiring:
// Pin 8 to IN1 on the ULN2003 driver
// Pin 9 to IN2 on the ULN2003 driver
// Pin 10 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver
#define IN1  8
#define IN2  9
#define IN3  10
#define IN4  11

#define LEDpin 6
#define buttonPin 2

// Define number of steps per rotation:
const int stepsPerRevolution = 4076;
// Direction of revolutions
int dir = 1;
// Trigger signal
bool trigger = false;
bool state = false;
bool output = false;
// counter 
int count = 0;

unsigned long lastTime, currentTime, difference;
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
  Timer1.initialize(400);
  Timer1.attachInterrupt(timerISR);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), takePhoto, RISING);
}

void loop() {
  currentTime = millis();
  difference = currentTime - lastTime;
  if ((digitalRead(buttonPin) == 1 || difference>5000) && trigger == false ){
    trigger = true;
    lastTime = currentTime;
  }
  if (stepper.distanceToGo() == 0){
    stepper.move(stepsPerRevolution*2);
  }
  stepper.run();
}

void timerISR(){
  if(trigger == true){
    state = !state;
    
    count += state;

    if(count <= 76){
      output = state;
    }
    if(count > 76 && count <= 1134){
      output = false;
    }
    if(count > 1134 && count <= 1149){
      output = state;
    }
    if(count > 1149 && count <= 1210){
      output = false;
    }
    if(count > 1210 && count <= 1225){
      output = state;
    }
    if(count > 1225 && count <= 1360){
      output = false;
    }
    if(count > 1360 && count <= 1375){
      output = state;
    }
    if(count > 1375){
      output = false;
      trigger = false;
      state = false;
      count = 0;
    }
    digitalWrite(13, output);
  }
}
