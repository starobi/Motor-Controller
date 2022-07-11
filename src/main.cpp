#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

//Pin defines
#define SENSORPWM1 PB7
#define SENSORPWM2 PB15

#define LEDPIN PA7
#define SLAVEADDRESS 0x8
#define SDAPIN PB9
#define SCLPIN PB8

#define SERVOSIGNAL PB6

//Hardware defines
#define SUPPLYVOLTAGE 10  
#define MOTORRESISTANCE 5.57

//Encoder range of values obtained experimentallz from Examples/Simple FOC/utils/sensor_test/magnetic_sensors/magnetic_sensor_pwm/find_raw_min_max
#define ENCODER1MINPULSE 6 
#define ENCODER1MAXPULSE 930

#define ENCODER2MINPULSE 9
#define ENCODER2MAXPULSE 898

#define POLEPAIRS 11
#define MAXTARGETVOLTAGE SUPPLYVOLTAGE
#define WHEELRADIUS 3.15

//Motor selection
#define USESERVO true
#define USEMOTOR1 true  
#define USEMOTOR2 true
#define CLOSEDLOOP true
#define MESSAGESIZE 8

void initMotors(void);
void initI2C(void); 
void blink(int amount, int del);
void requestI2C(void);
void receiveI2C (int bytes);


//Servomotor PWM configuration. Code taken from Examples/STM32duino Examples/Peripherals/Hardware Timer/ All-in-one-PWM and  PWM_Full_Configuration
////////////////////////////////////////
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(SERVOSIGNAL), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(SERVOSIGNAL), PinMap_PWM));
HardwareTimer *PWMTimer = new HardwareTimer(Instance);
////////////////////////////////////////

//Motor, driver and sensor instances
BLDCMotor motor1 = BLDCMotor(POLEPAIRS, MOTORRESISTANCE);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA0, PA1, PA2, PC14);
MagneticSensorPWM sensor1 = MagneticSensorPWM(SENSORPWM1, ENCODER1MINPULSE, ENCODER1MAXPULSE);
void doPWM1(){sensor1.handlePWM();}

BLDCMotor motor2 = BLDCMotor(POLEPAIRS, MOTORRESISTANCE);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PC6, PC7, PC8, PC15);
MagneticSensorPWM sensor2 = MagneticSensorPWM(SENSORPWM2, ENCODER2MINPULSE, ENCODER2MAXPULSE);
void doPWM2(){sensor2.handlePWM();}

//I2C message array
char message[MESSAGESIZE];
//Target voltage is set via I2C receive and correlates to the motors RPM
float motorTargetVoltage = 0;
//Servo is moved via this PWM variable
float servoPulseWidth = 1500;

//Setup code, run once
void setup() {
  blink(2, 100);

  initI2C();
  initMotors();

  blink(3, 200);  
}

//Main code, loops constantly
//loopFOC checks rotor position and currents
//move spins motor defined by voltage
void loop() {
  if (USEMOTOR1) {
    motor1.loopFOC();  
  }
  if (USEMOTOR2) {
    motor2.loopFOC();  
  }

  if (USEMOTOR1) {
    motor1.move(motorTargetVoltage);
  }
  if (USEMOTOR2) {
    motor2.move(motorTargetVoltage);
  }
}

//blinks  ¯\_(ツ)_/¯
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
void PWMTimerCallback(void) { 
if (USESERVO) {
    PWMTimer->setCaptureCompare(channel,servoPulseWidth,MICROSEC_COMPARE_FORMAT);
  }
}

//initializes I2C connection to Raspi
void initI2C() {
  Wire.setSDA(SDAPIN);
  Wire.setSCL(SCLPIN);
  Wire.begin(SLAVEADDRESS);
  Wire.onReceive(receiveI2C);
  Wire.onRequest(requestI2C);
}

//initializes motors, drivers and sensors
void initMotors() {
  //servo init
  if (USESERVO) {
    pinMode(PB10, OUTPUT);
    PWMTimer->setPWM(channel, SERVOSIGNAL, 50, 7.5); // 50 Hertz, 7.5% dutycycle or Pulse Width of 1.5 ms
    PWMTimer->attachInterrupt(PWMTimerCallback);
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
    motor1.initFOC(); //Values obtained experimentaly from Examples/Simple FOC/utils/calibration/find_sensor_offset_and_direction
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
    motor2.initFOC(); //Values obtained experimentaly from Examples/Simple FOC/utils/calibration/find_sensor_offset_and_direction
 }
}

//receives data via I2C
//byte 2 and 3 are speed and steering, which get denormalized and stored
void receiveI2C (int bytes)
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
  //servo values must be between 2000 us and 1000 us
  servoPulseWidth = (-3.921569)*str+2000; 
}

//sends data upon I2C request
void requestI2C()
{
  // Convert from angular rad/s to linear velocity cm/s
  float vel = (motor1.shaft_velocity + motor2.shaft_velocity / 2) * WHEELRADIUS;
  Wire.write((int)vel);
}