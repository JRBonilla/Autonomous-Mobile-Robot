#include <ServoTimer2.h>

//========================================================
// Constant variables
//========================================================
const int LIGHT_THRESHOLD = 500;
const int MAX_DISTANCE    = 11;
const int MAX_HEIGHT      = 20;
const int MAX_SPEED       = 128;

const int INIT_POSITION   = 375;
const int FINAL_POSITION  = 1925;
const int MIDDLE_POSITION = 1500;
const int NUM_POSITIONS   = 4;

//========================================================
// Motor & other variables
//========================================================
int buttonPin        = 13; // Pin D13
int buzzerPin        = 12; // Pin D12

int left_motor_en    = 10; // Pin D10
int left_motor_back  = 9;  // Pin D9
int left_motor_go    = 8;  // Pin D8

int right_motor_back = 7;  // Pin D7
int right_motor_go   = 6;  // Pin D6
int right_motor_en   = 5;  // Pin D5

//=========================================================
// Ultrasonic edge detection and object avoidance variables
//=========================================================
int EdgeRightEcho = A1;  // Set right Echo port
int EdgeRightTrig = 4;   // Set right Trig port
int EdgeRightDist = 0;

int EdgeLeftEcho = A5;   // Set left Echo port
int EdgeLeftTrig = A4;   // Set left Trig port
int EdgeLeftDist = 0;

int AvoidEcho = A2;      // Set front facing echo port
int AvoidTrig = A3;      // Set front facing trig port
int AvoidDist = 0;

// Returns whether or not the edge sensors are currently off the edge.
bool isLeftOOB    = (EdgeRightDist < MAX_HEIGHT && EdgeLeftDist > MAX_HEIGHT); // Left sensor out of bounds
bool isRightOOB   = (EdgeRightDist > MAX_HEIGHT && EdgeLeftDist < MAX_HEIGHT); // Right sensor out of bounds
bool isBothOOB    = (EdgeRightDist > MAX_HEIGHT && EdgeLeftDist > MAX_HEIGHT); // Both sensors out of bounds
bool isOnPlatform = (EdgeRightDist < MAX_HEIGHT && EdgeLeftDist < MAX_HEIGHT); // Neither sensor out of bounds

//=========================================================
// Servo variables
//=========================================================
int ServoPort = 0;  // Servo port
ServoTimer2 servo;  // Servo

//=========================================================
// IR sensor variables
//=========================================================
const int IRSensorRight = 1;  // IR sensor right port (D1)
const int IRSensorLeft  = 11; // IR sensor left port (D11)
int SL, SR;                   // IR sensor states

//=========================================================
// Light sensor variables
//=========================================================
int LightSensorPin = A0;              // Light sensor port
int LightSensorVal = 0;               // Current light sensor value
int LightSensorValues[NUM_POSITIONS]; // Light sensor values for the different servo positions

// Initial setup code
void setup() {
  // Begin serial output
  Serial.begin(9600); // Do not uncomment this unless you want debugging and aren't using D0 and/or D1
  
  // Initialize left and right motor drive for output mode.
  pinMode(left_motor_go,    OUTPUT);
  pinMode(left_motor_back,  OUTPUT);
  pinMode(right_motor_go,   OUTPUT);
  pinMode(right_motor_back, OUTPUT);
  pinMode(right_motor_en,   OUTPUT);
  pinMode(left_motor_en,    OUTPUT);

  pinMode(buttonPin, INPUT);       // Set button as input
  //pinMode(beep, OUTPUT);         // Set buzzer as output

  pinMode(EdgeRightEcho, INPUT);   // Set Echo port mode
  pinMode(EdgeRightTrig, OUTPUT);  // Set Trig port mode

  pinMode(EdgeLeftEcho, INPUT);    // Set Echo port mode
  pinMode(EdgeLeftTrig, OUTPUT);   // Set Trig port mode

  pinMode(AvoidEcho, INPUT);       // Set Echo port mode
  pinMode(AvoidTrig, OUTPUT);      // Set Trig port mode

  servo.attach(ServoPort);         // Attach the servo to the servo port

  digitalWrite(buttonPin, HIGH);   // Initialize button
  //digitalWrite(buzzerPin, LOW);  // Set buzzer mute

  pinMode(IRSensorRight, INPUT);   // Set right Line Walking IR sensor as input
  pinMode(IRSensorLeft,  INPUT);   // Set left Line Walking IR sensor as input

  pinMode(LightSensorPin, INPUT);  // Set lightsensor as input
}

// Move forward
void run() {
  digitalWrite(left_motor_en, HIGH);   // Left motor enable
  analogWrite(left_motor_en, 130);
  digitalWrite(right_motor_en, HIGH);  // Right motor enable

  digitalWrite(right_motor_go, HIGH);  // Right motor go ahead
  digitalWrite(right_motor_back, LOW);
  analogWrite(right_motor_go, 130);    // PWM--Pulse Width Modulation (0~255). right motor go speed is 130
  analogWrite(right_motor_back, 0);

  digitalWrite(left_motor_go, HIGH);   // Left motor go ahead
  digitalWrite(left_motor_back, LOW);
  analogWrite(left_motor_go, 130);     // PWM--Pulse Width Modulation (0~255). left motor go speed is 130
  analogWrite(left_motor_back, 0);
}

// Stop
void brake() {
  digitalWrite(right_motor_go,   LOW); // Stop the right motor
  digitalWrite(right_motor_back, LOW);

  digitalWrite(left_motor_go,    LOW); // Stop the left motor
  digitalWrite(left_motor_back,  LOW);
}

// Turn left
void left() {
  digitalWrite(right_motor_go, HIGH);  // Right motor go ahead
  digitalWrite(right_motor_back, LOW);
  analogWrite(right_motor_go, 190);    // PWM--Pulse Width Modulation(0~255) control speed，right motor go speed is 190.
  analogWrite(right_motor_back, 0);
  
  digitalWrite(left_motor_go, LOW);    // Left motor stop
  digitalWrite(left_motor_back, LOW); 
  analogWrite(left_motor_go, 0); 
  analogWrite(left_motor_back, 0);
  delay(150); 
}

// Rotate left
void spin_left(int time) {
  digitalWrite(right_motor_go,  HIGH);     // Right motor go ahead
  digitalWrite(right_motor_back, LOW);
  analogWrite(right_motor_go, MAX_SPEED);  // PWM--Pulse Width Modulation(0~255) control speed, right motor go speed is 200
  analogWrite(right_motor_back, 0);

  digitalWrite(left_motor_go,   LOW);      // Left motor back off
  digitalWrite(left_motor_back, HIGH);
  analogWrite(left_motor_go, 0);
  analogWrite(left_motor_back, MAX_SPEED); // PWM--Pulse Width Modulation(0~255) control speed, left motor back speed is 200.

  delay(time * 100);
}

// Turn right
void right() {
  digitalWrite(right_motor_go, LOW);   // Right motor stop
  digitalWrite(right_motor_back, LOW);
  analogWrite(right_motor_go, 0); 
  analogWrite(right_motor_back, 0);
  
  digitalWrite(left_motor_go, HIGH);   // Left motor go ahead
  digitalWrite(left_motor_back, LOW);
  analogWrite(left_motor_go, 230);     // PWM--Pulse Width Modulation(0~255) control speed ,left motor go speed is 230.
  analogWrite(left_motor_back, 0);
  delay(150);
}

// Rotate right
void spin_right(int time) {
  digitalWrite(right_motor_go,    LOW);     // Right motor back off
  digitalWrite(right_motor_back, HIGH);
  analogWrite(right_motor_go, 0);
  analogWrite(right_motor_back, MAX_SPEED); // PWM--Pulse Width Modulation(0~255) control speed

  digitalWrite(left_motor_go,  HIGH);       // Left motor go ahead
  digitalWrite(left_motor_back, LOW);
  analogWrite(left_motor_go, MAX_SPEED);    // PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(left_motor_back, 0);

  delay(time * 100);
}

// Go backwards
void back(int time) {
  digitalWrite(right_motor_go,    LOW);     // Right motor back off
  digitalWrite(right_motor_back, HIGH);
  analogWrite(right_motor_go, 0);
  analogWrite(right_motor_back, MAX_SPEED); // PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(right_motor_en, 200);

  digitalWrite(left_motor_go,    LOW);      // Left motor back off
  digitalWrite(left_motor_back, HIGH);
  analogWrite(left_motor_go, 0);
  analogWrite(left_motor_back, MAX_SPEED);  // PWM--Pulse Width Modulation(0~255) control speed

  delay(time * 100);
}

// Right ultrasonic sensor distance test
void EdgeRightDistanceTest() {
  digitalWrite(EdgeRightTrig, LOW);               // Set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(EdgeRightTrig, HIGH);              // Set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(EdgeRightTrig, LOW);               // Set trig port low level
  float Fdistance = pulseIn(EdgeRightEcho, HIGH); // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;                        // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                                  //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                                  //  ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  //Serial.print("Edge right Distance:");           // Output Distance(cm)
  //Serial.println(Fdistance);                      // Display distance
  EdgeRightDist = Fdistance;
}  

// Left ultrasonic sensor distance test
void EdgeLeftDistanceTest() {
  digitalWrite(EdgeLeftTrig, LOW);               // Set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(EdgeLeftTrig, HIGH);              // Set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(EdgeLeftTrig, LOW);               // Set trig port low level
  float Fdistance = pulseIn(EdgeLeftEcho, HIGH); // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;                       // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                                 //  ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  //Serial.print("Edge left Distance:");           // Output Distance(cm)
  //Serial.println(Fdistance);                     // Display distance
  EdgeLeftDist = Fdistance;
}  

// Object avoidance sensor distance test
void AvoidDistanceTest() {
  digitalWrite(AvoidTrig, LOW);               // Set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(AvoidTrig, HIGH);              // Set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(AvoidTrig, LOW);               // Set trig port low level
  float Fdistance = pulseIn(AvoidEcho, HIGH); // Read echo port high level time(unit:μs)
  Fdistance= Fdistance/58;                    // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                              //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                              //  ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  //Serial.print("Avoid Distance:");            // Output Distance(cm)
  //Serial.println(Fdistance);                  // Display distance
  AvoidDist = Fdistance;
}

// Returns whether or not the object avoidance sensor detected an object
boolean isObjectInFront(){
    AvoidDistanceTest(); // Check the avoidance sensor distance once.

    int old = AvoidDist; // Save the prior result and wait 250 ms before the next test.
    delay(250);
    
    AvoidDistanceTest(); // Check the avoidance sensor distance again.
    
    // If both checks are less than MAX_DISTANCE then there is something in front
    if (AvoidDist < MAX_DISTANCE && old < MAX_DISTANCE ) {
      return true;
    }
    return false;
}

// Edge detection code
void EdgeCheck(){
    EdgeRightDistanceTest(); // Measuring right ultrasonic distance
    EdgeLeftDistanceTest();  // Measuring left ultrasonic distance
    
    if (EdgeRightDist > MAX_HEIGHT && EdgeLeftDist > MAX_HEIGHT) {  // Both are out of bounds
      back(2);
      spin_right(6);
    }
    if (EdgeRightDist < MAX_HEIGHT && EdgeLeftDist > MAX_HEIGHT) {  // Right is out of bounds
      back(2);
      spin_left(6);
    }
    if (EdgeRightDist > MAX_HEIGHT && EdgeLeftDist < MAX_HEIGHT) {  // Left is out of bounds
      back(2);
      spin_right(6);
    }
}

// Store the light sensor value based on the current servo position.
void StoreLightSensorValue(int pos) {
  LightSensorVal = analogRead(LightSensorPin);
  if (pos < NUM_POSITIONS) {
    LightSensorValues[pos] = LightSensorVal;
  }
}

// Go in the direction with the brightest light
void ChoosePath() {
  // Find the brightest direction
  int minimum = LightSensorValues[0];
  for (int i = 0; i < NUM_POSITIONS; i++) {
    if (LightSensorValues[i] < minimum) {
      minimum = LightSensorValues[i];
    }
  }

  switch (minimum) {
    case 0:  // Brightest direction is on the right
      break;
    case 1:  // Brightest direction is 
      break;
    case 2:  // Brightest direction is 
      break;
    case 3:  // Brightest direction is on the left
      break;
  }
}

// Returns whether or not there is a candle in front of the robot.
boolean isCandlePresent() {
  LightSensorVal = analogRead(LightSensorPin);

  if (LightSensorVal < LIGHT_THRESHOLD) {
    Serial.print("Candle is present: ");
    Serial.println(LightSensorVal);
    return true;
  }
  Serial.print("No candle present: ");
  Serial.println(LightSensorVal);
  return false;
}

// Check for button press
void keyscan() {
  int val = digitalRead(buttonPin);     // Reads the button, the level value assigns to val.
  while (digitalRead(buttonPin)) {      // When the button is not pressed
    val = digitalRead(buttonPin);
  }
  while (!digitalRead(buttonPin)) {     // When the button is pressed
    delay(10); // delay 10ms
    val = digitalRead(buttonPin);       // Read
    if (val == LOW) {                   // Double check the button is pressed
      digitalWrite(buzzerPin, HIGH);
      delay(50);                        // Delay 50ms
      while (!digitalRead(buttonPin)) { // Determine if button is released or not
        digitalWrite(buzzerPin, LOW);   // Mute
      }
    }
    else {
      digitalWrite(buzzerPin, LOW);     // Mute
    }
  }
}

void loop() {
  keyscan(); // Press the button to start
  while (true) {
    // Object avoidance code
    int servoPosition = 0;         // Tracks servo position
    for (int i = INIT_POSITION; i <= FINAL_POSITION; i += 375){
      servo.write(i);
      brake();
      delay(100);
      
      EdgeCheck();
      StoreLightSensorValue(servoPosition);
      
      if (isObjectInFront()) {     // There is an object in front of the robot.
        if (i <= MIDDLE_POSITION){ // The sensor is facing the right.
          spin_left(5);
        }
        else {
          spin_right(5);           // The sensor is facing the left.
        }
        i = INIT_POSITION;         // Reset the servo position.
        servoPosition = 0;         // Reset the position tracker.
        //ChoosePath();
      }
      servoPosition++;
    }
    
    // Black line following code
    EdgeRightDistanceTest(); // Measuring right ultrasonic distance
    EdgeLeftDistanceTest();  // Measuring left ultrasonic distance
    
    SL = digitalRead(IRSensorLeft);  // Read left IR sensor state
    SR = digitalRead(IRSensorRight); // Read right IR sensor state
    if (SL == LOW && SR == LOW && isOnPlatform) {        // No black lines detected
      run();
      delay(60);
      brake();
      delay(100);
    }
    else if (SL == LOW && SR == HIGH) {                  // Black line detected on the right, turn right
      right();
    }
    else if (SL == HIGH && SR == LOW) {                  // Black line detected on the left, turn left
      spin_left(1);
    }
    else if (SL == HIGH && SR == HIGH && isOnPlatform) { // Black line detected on both left and right, turn right
      spin_left(3);
    }

    // Edge detection
    EdgeCheck();
  }

}
