#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>


/*
   Rotary Encoder Wires:
   Purple - PWM -  Gray
   Green  - GND -  Black
   Blue   - 5V  -  White
*/

#define SENSORPWM1 PB7
#define SENSORPWM2 PB15

#define LEDPIN PA7
#define SLAVEADDRESS 0x8
#define SDAPIN PB9
#define SCLPIN PB8

#define SERVOSIGNAL PB6

#define SUPPLYVOLTAGE 10  
#define MOTORRESISTANCE 5.57

#define SENSORMINPULSE 6
#define SENSORMAXPULSE 940

#define POLEPAIRS 11
#define CLOSEDLOOP true

#define MESSAGESIZE 8
#define MAXTARGETVOLTAGE SUPPLYVOLTAGE

//Options to activate and deactivate motors
#define USESERVO true
#define USEMOTOR1 true  
#define USEMOTOR2 true

void requestFun(void);
void receiveFun (int bytes);
void initMotors(void);
void initI2C(void); 
void blink(int amount, int del);

//Servomotor PWM configuration. Code taken from Examples/STM32duino Examples/Peripherals/Hardware Timer/ All-in-one-PWM and  PWM_Full_Configuration
////////////////////////////////////////
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(SERVOSIGNAL), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(SERVOSIGNAL), PinMap_PWM));
HardwareTimer *MyTim = new HardwareTimer(Instance);
////////////////////////////////////////


BLDCMotor motor1 = BLDCMotor(POLEPAIRS, MOTORRESISTANCE);//, MOTORRESISTANCE);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA0, PA1, PA2, PC14);
MagneticSensorPWM sensor1 = MagneticSensorPWM(SENSORPWM1, SENSORMINPULSE, SENSORMAXPULSE);
void doPWM1(){sensor1.handlePWM();}

BLDCMotor motor2 = BLDCMotor(POLEPAIRS, MOTORRESISTANCE);//, MOTORRESISTANCE);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PC6, PC7, PC8, PC15);
MagneticSensorPWM sensor2 = MagneticSensorPWM(SENSORPWM2, SENSORMINPULSE, SENSORMAXPULSE);
void doPWM2(){sensor2.handlePWM();}

char message[MESSAGESIZE];
float motorTargetVoltage = 0;
float servoDC = 1500;

void setup() {
  blink(2, 100);

  initI2C();
  initMotors();

  blink(3, 200);  
}

void loop() {
  // main FOC algorithm function
  if (USEMOTOR1){
    motor1.loopFOC();  
  }
  if (USEMOTOR2){
    motor2.loopFOC();  
  }

  if (USEMOTOR1){
  motor1.move(motorTargetVoltage);
  }
  if (USEMOTOR2){
  motor2.move(motorTargetVoltage);
  }
}
void blink(int amount, int del) {
  pinMode(LEDPIN, OUTPUT);

  for (int i = 0; i < amount; i++) {
    digitalWrite(LEDPIN, HIGH);
    delay(del);
    digitalWrite(LEDPIN, LOW);
    delay(del);
  }
}
//Interrupted on rising edge Servo-PWM-pin
void Update_IT_callback(void){ 
  
  if (USESERVO)
  {
    MyTim->setCaptureCompare(channel,servoDC,MICROSEC_COMPARE_FORMAT);
  }
  
}

void initI2C() {
  Wire.setSDA(SDAPIN);
  Wire.setSCL(SCLPIN);
  Wire.begin(SLAVEADDRESS);
  Wire.onReceive(receiveFun);
  Wire.onRequest(requestFun);
}

void initMotors() {

  if (USESERVO)
  {
      pinMode(PB10, OUTPUT);
  MyTim->setPWM(channel, SERVOSIGNAL, 50, 7.5); // 50 Hertz, 7.5% dutycycle
  MyTim->attachInterrupt(Update_IT_callback);
  }

  
  pinMode(SENSORPWM1, INPUT);
  pinMode(SENSORPWM2, INPUT);
  pinMode(PC3, OUTPUT);
  pinMode(PA3, OUTPUT);
  digitalWrite(PC3, HIGH);
  digitalWrite(PA3, HIGH);

  if (USEMOTOR1){
      sensor1.init();
      sensor1.enableInterrupt(doPWM1);
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = SUPPLYVOLTAGE;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor1.init();
  motor1.initFOC(3.2, CW); //Values obtained experimentaly from Examples/Simple FOC/utils/calibration/alignment_and_cogging_test
  }

  
 if (USEMOTOR2) {
    sensor2.init();
    sensor2.enableInterrupt(doPWM2);
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = SUPPLYVOLTAGE;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.controller = MotionControlType::torque;
  motor2.init();
  motor2.initFOC(4.8,CCW); //Values obtained experimentaly from Examples/Simple FOC/utils/calibration/alignment_and_cogging_test
 }

}

void receiveFun (int bytes)
{
  int i = 0;
  while (Wire.available() && i < MESSAGESIZE)
  {
    message[i] = Wire.read();
    i++;
  }

  float spd = message[1];
  float str = message[2];

  //de-normalize values
  spd = spd - 128;
  spd = spd / 128 * MAXTARGETVOLTAGE;
  motorTargetVoltage = spd;
  servoDC = (-3.921569)*str+2000; //The values must be between 2000 us and 1000 us
}

void requestFun()
{
  // Convert from angular rad/s to linear velocity m/s
  // TODO calculate correct conversion factor
  float vel = sensor1.getVelocity() + sensor2.getVelocity() / 2 * 5;
  // TODO check I2C float specs, convert if neccessary
  Wire.write((int)vel);
}
