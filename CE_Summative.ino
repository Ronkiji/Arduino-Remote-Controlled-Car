/*
   CE Summative - Joystick controlled car using continuous
   servo motors including a sensor at the front.
   By Ronald Sun
   January 11, 2020
   Three options - joystick control, auto run mode, table mode
*/

#include <Servo.h> // Servo library
#include <IRremote.h> // Remote control library

Servo servo; // declare a servo

/* Connections for the two motors - these are connected to the H bridge */
#define motorA1 4 // left wheel
#define motorA2 5
#define motorB1 7 // right wheel
#define motorB2 6

/* Defining pin numbers */
#define servoPin 2
#define echoPin 8
#define trigPin 9
#define remotePin 12

IRrecv receiver(remotePin); // receiver for remote
decode_results results;

/* Speed for both motors */
int speedA = 0;
int speedB = 0;

int distance; // distance measured on the ultrasonic sensor

int mode = 1; // the selected mode - out of three different modes

int pos;

void setup() {
  Serial.begin(9600);

  receiver.enableIRIn();

  pinMode(motorA1, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(10, OUTPUT); // green LED
  pinMode(11, OUTPUT); // red LED

  servo.attach(servoPin);
  servo.write(102); // default upright position of the servo motor
  pos = 102;
}

void loop() {

  switch (mode) {
    case 1:
      joystickControl();
      break;
    case 2:
      modeTwo();
      break;
    case 3:
      modeThree();
      break;
  }
  
  if (receiver.decode(&results)) { // the following bit of code for the receiver is modified from
    // https://create.arduino.cc/projecthub/electropeak/use-an-ir-remote-transmitter-and-receiver-with-arduino-1e6bc8
    int value = results.value;
    switch (value) {
      
      case 12495: // Keypad button "1"
        Serial.println("position 1");
        Serial.println(pos);
        mode = 1;
        servo.write(102);
        for (pos; pos < 102; pos++) {
          servo.write(pos + 1); // servo upright
          delay(5);
        }
        break;
        
      case 6375: // Keypad button "2"
        Serial.println("position 2");
        Serial.println(pos);
        mode = 2;

        /* Momentary stop */
        analogWrite(motorA2, 0);
        analogWrite(motorB2, 0);

        for (pos; pos < 102; pos++) {
          servo.write(pos + 1); // servo upright
          pos += 1;
          delay(50);
        }
        delay(2000); // give it two seconds before it starts
        break;
        
      case 31365: // Keypad button "3"
        Serial.println("position 3");
        Serial.println(pos);
        mode = 3;
        /* Momentary stop */
        analogWrite(motorA2, 0);
        analogWrite(motorB2, 0);
        for (pos; pos > 25; pos--) { // servo pointing down to measure distance between surface and sensor
          servo.write(pos - 1); // servo facing down
          delay(50);
        }
        delay(2000); // give it two seconds before it starts
        break;
      
    }
    receiver.resume();
  }
}

void modeOne() { // joystick controlled car - if the motor detects anything within 10cm, it will turn 90 degrees right
//  if (millis() % 200 == 0) { // to not let it read every millisecond
//    readDistance();
//    if (distance <= 10) {
//      readDistance();// to ensure that its below 5 for at least two tics
//      // so that it doesnt spin around accidentally
//      if (distance <= 10) {
//
//        turn();
//        digitalWrite(10, LOW); // green LED off
//        delay(500); // time for it to spin
//
//      }
//    }
//    digitalWrite(11, LOW); // red LED off
//  }
  joystickControl(); // this loop will allow joystick control
}

void modeTwo() { // autorun car that will turn whenever distance gets close
  readDistance();
  if (distance < 9) {
    readDistance();
    if (distance < 9) { // ensure distance is below 9 for two tics
      while (distance < 15) {
        turn(); // will spin until distance in front of car is cleared
        readDistance();
      }
    }
  } else {
    autoRun(); // run forward
  }
}

void modeThree() { // autorun car running on a table, and will turn whenever it reaches an edge so that it will not fall off
  readDistance();
  if (distance > 9) {

    /* STOPPING MOTORS IMMEDIATELY */
    analogWrite(motorA2, 0);
    analogWrite(motorB2, 0);
    delay(500); // its on a table - so this is just a safety precaution

    while (distance >= 9) {
      turn(); // will spin until distance in front of car is cleared
      readDistance();
    }
  } else {
    autoRun(); // run forward
  }
}

void readDistance() { // Code below referenced from https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
}

void joystickControl() {

  int VRx = analogRead(A4); // get x axis location
  int VRy = analogRead(A5); // get y axis location

  /* Y axis movement that will declare forward/back movement */
  if (VRy < 480) { // axis pointed forward

    /* Settings motors to run forward */
    digitalWrite(motorA1, LOW);
    digitalWrite(motorB1, HIGH);

    /* Converting the y axis position from 480 - 0 to the
       range of speed 0 - 255 that is applicable to the motor
       map(value, fromLow, fromHigh, toLow, toHigh)          */
    speedA = map(VRy, 480, 0, 0, 255);
    speedB = map(VRy, 480, 0, 0, 255);

  } else if (VRy > 540) { // axis pointed backward

    /* Settings motors to run forward */
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorB1, LOW);

    /* Converting the y axis position from 540 - 1023 to the
       range of speed 0 - 255 that is applicable to the motor
       map(value, fromLow, fromHigh, toLow, toHigh) */
    speedA = map(VRy, 540, 1023, 0, 255);
    speedB = map(VRy, 540, 1023, 0, 255);

  } else { // to prevent buzzing, speed will be 0 if joystick is around center
    speedA = 0;
    speedB = 0;
  }

  /* X axis movement that will declare left/right movement
     The logic behind the turning mechanism is to proportionally have
     one wheel go faster and the other slower. By mapping the speed,
     that will be used to change the speed of both motors. */
  if (VRx < 480) {
    // Converting the x axis to fit the range of 0 - 255
    int xPos = map(VRx, 480, 0, 0, 255);

    speedA = speedA - xPos; // decrease left motor speed
    speedB = speedB + xPos; // increase right motor speed

    /* Ensuring the ranges does not surpass the min and max speed of 0 - 255 */
    if (speedA < 0)
      speedA = 0;
    else if (speedB > 255)
      speedB = 255;

  } else if (VRx > 540) {
    // Converting the x axis to fit the range of 0 - 255
    int xPos = map(VRx, 540, 1023, 0, 255);

    speedA = speedA + xPos; // increase left motor speed
    speedB = speedB - xPos; // decrease right motor speed

    /* Ensuring the ranges does not surpass the min and max speed of 0 - 255 */
    if (speedB < 0)
      speedB = 0;
    else if (speedA > 255)
      speedA = 255;

  }

  // Prevent buzzing at low speeds
  if (speedA < 60) {
    speedA = 0;
  }
  if (speedB < 60) {
    speedB = 0;
  }

  if (speedA > 60 || speedB > 60) // turn on green LED when motor running
    digitalWrite(10, HIGH);
  else
    digitalWrite(10, LOW);

  /* Sends the motors a Pulse Width Modulation (PWM) that ranges
     from 0 - 255 which will decide the speed in which the motor runs */
  analogWrite(motorA2, speedA);
  analogWrite(motorB2, speedB);
}

void autoRun() {
  digitalWrite(10, HIGH); // green led on
  digitalWrite(11, LOW); // red LED off

  digitalWrite(motorA1, LOW); // forward
  digitalWrite(motorB1, HIGH); // forward

  // slower speed in order to safely autorun the car
  analogWrite(motorA2, 100);
  analogWrite(motorB2, 100);
}

void turn() {
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);

  /* Turning right */
  digitalWrite(motorA1, LOW); // forward
  digitalWrite(motorB1, LOW); // backward

  analogWrite(motorA2, 150);
  analogWrite(motorB2, 150);
}
