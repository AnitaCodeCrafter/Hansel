#define MAX_SEGMENTS 20

long distanceLog[MAX_SEGMENTS];   // encoder pulses for each straight section
float turnLog[MAX_SEGMENTS];      // angle for each turn

int segmentIndex = 0;

long segmentStartCount = 0;
float segmentStartAngle = 0;

//reverse turns
#include <Wire.h>

// MPU-6500
#define MPU_ADDR 0x68

float gyroZ = 0;
float angleZ = 0;        // total accumulated angle
float returnAngle = 0;   // angle to reverse
unsigned long prevTime = 0;
//calibrating
float gyroZ_offset = 0;

//return backwards/forward
long targetPulses = 0;
bool returning = false;

//encoder
#define encoder 2
volatile long countForward = 0;
volatile long countBackward = 0;
volatile long returningCount = 0;
bool forward = false;
bool backward = false;
bool turning = false;
/*
const int PULSES_PER_REV = 21;     // change to your encoder spec
const float WHEEL_DIAMETER = 6.0; // cm
const float PI_VAL = 3.1416;
*/
// remote control car
#include<IRremote.h>
const int RemotePin=8;
IRrecv irrecv(RemotePin);
decode_results results;
int in1=3;
int in2=5;
int in3=6;
int in4=9;

void setup() {
Serial.begin(115200);
//gyroscope
  setupMPU();
  prevTime = millis();

  delay(2000);        // let sensor stabilize
  calibrateGyro();    // measure bias
  prevTime = millis();

//encoder
  pinMode(encoder, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder), pulse, RISING); // whenever pin 2 sees a RISING edge, immediately run the function pulse()

//remote control car
  IrReceiver.begin(RemotePin, ENABLE_LED_FEEDBACK);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.println("IR Car Ready");
 
}

void loop() {
  //Backward();
  if (turning){
    updateGyro();
  }

  
//counting distance
/*
  noInterrupts();
  long pulses = count;
  interrupts();

  float turns = (float)pulses / PULSES_PER_REV;
  float distance = turns * PI_VAL * WHEEL_DIAMETER;

    Serial.print("Pulses: ");
    Serial.print(pulses);
    Serial.print(" | Distance (cm): ");
    Serial.println(distance);

  delay(200);
*/

// remote control car
  if (IrReceiver.decode()) {

    uint32_t irValue = IrReceiver.decodedIRData.decodedRawData;

    Serial.print("HEX: ");
    Serial.println(irValue, HEX);

    if (irValue == 0xB946FF00) {       
      Forward();
    }
    else if (irValue == 0xEA15FF00) {  
      Backward();
    }
    else if (irValue == 0xBB44FF00) {    
      Left();
    }
    else if (irValue == 0xBC43FF00) {    
      Right();
    }
    else if (irValue == 0xB847FF00) {    
      Stop();
    }
    else if (irValue == 0xF20DFF00) { // fill in value for return button   
      Reverse();
    }

    IrReceiver.resume();
  }
}

void pulse() { 
  if (returning) {
    returningCount++;
  }
  else if (forward) {
    countForward++;
  }
  else if (backward) {
    countBackward++;
  }
}   
 
void Reverse() {
  Stop(); // This saves the very last segment
  returning = true;

  // Work backwards through the saved segments
  for (int i = segmentIndex - 1; i >= 0; i--) {
    // 2. UNDO THE DISTANCE
    returningCount = 0;
    while (returningCount < abs(distanceLog[i])) {
      if (distanceLog[i] > 0) {
        Backward(); // If we went forward, go backward
      } else {
        Forward(); // If we went backward, go forward
      }
      // Note: your pulse() function handles returningCount++
      delay(5);
    }
    Stop();
    delay(200);
  // 1. UNDO THE TURN FIRST
    angleZ = 0;
    prevTime = millis(); // Reset gyro timer
    while (abs(angleZ) < abs(turnLog[i])) {
      if (turnLog[i] > 0) {
        Right(); // If we turned right originally (+), turn left to go back
      } else {
        Left(); // If we turned left originally (-), turn right to go back
      }
      updateGyro();
    }
    Stop();
    delay(200);
  }
  
  returning = false;
  segmentIndex = 0;
}

void Backward() {
  prevTime = millis();
    //Serial.println("Backward");
    backward = true;
    forward = false;
    turning = false;

    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
void Forward() {
  prevTime = millis();
    //Serial.println("Forward");
    forward = true;
    backward = false;
    turning = false;

    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
}
void Stop() {
  if (!returning) { // Only save segments during recording, not playback
    saveSegment();
  }
  forward = false;
  backward = false;
  turning = false;

  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
  void Left() {
  prevTime = millis();
  if (!turning && !returning) { // Added !returning here
    saveSegment();
  }
  turning = true;
      //Serial.println("Left");
    if (backward == true){
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
      digitalWrite(in3,LOW);
      digitalWrite(in4,LOW);
    }
    else{
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
      digitalWrite(in3,LOW);
      digitalWrite(in4,LOW);
    }
    
    }
  void Right() {
    prevTime = millis();
    if (!turning && !returning) { // Added !returning here
      saveSegment();
    }
    turning = true;
      //Serial.println("Right");
    /*if (returning && forward == true){
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
    }*/
    if (backward == true){
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
    }
    else{
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
    }
  }

  void setupMPU() {
    Wire.begin();

    // Wake up MPU-6500
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);    
    Wire.endTransmission(true);

    // Set gyro full scale ±250 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void updateGyro() {
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);  // Gyro Z high byte
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t rawZ = Wire.read() << 8 | Wire.read();

  gyroZ = (rawZ / 131.0) - gyroZ_offset;  // sensitivity for ±250 deg/s
  if (abs(gyroZ) < 7) gyroZ = 0;// throw out values that are very small

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  angleZ += gyroZ;
  Serial.print("AngleZ = ");
  Serial.print(angleZ);
  Serial.print("gyroZ = ");
  Serial.println(gyroZ);

}

void calibrateGyro(){

  Serial.println("Calibrating gyro... Keep car still.");

  long sum = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++){

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);   // Gyro Z register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);

    int16_t rawZ = Wire.read() << 8 | Wire.read();
    sum += rawZ;

    delay(3);  // small delay between samples
  }

  gyroZ_offset = (sum / (float)samples) / 131.0;

  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZ_offset);

  Serial.println("Calibration complete.");
}

void saveSegment() {
  if (segmentIndex >= MAX_SEGMENTS) return;

  // Save the distance moved since the last state change
  distanceLog[segmentIndex] = countForward - countBackward;
  
  // Save the angle turned since the last state change
  turnLog[segmentIndex] = angleZ;

  Serial.print(" Save Segment index = ");
  Serial.print(segmentIndex);
  Serial.print(" Distance = ");
  Serial.print(distanceLog[segmentIndex]);
  Serial.print(" angle = ");
  Serial.println(turnLog[segmentIndex]);

  // Reset EVERYTHING for the next segment
  countForward = 0;
  countBackward = 0;
  angleZ = 0; // Reset angle so each segment is relative

  prevTime = millis();
  segmentIndex++;
}