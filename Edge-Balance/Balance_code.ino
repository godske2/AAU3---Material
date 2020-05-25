#include <stdlib.h>
#include <Wire.h>

#define CURRENT_MAX 7.5           //max current per motor
#define recovery 40*3.14/180.0    //The AAU3 has fallen
#define rpm2rad 2.0*3.14/60.0     //Conversion from RPM to Radian/s
#define CHECK_CYCLE 50
#define reject 10
#define step_size 0.1*0.01
#define large_step_size 0.1*0.1
#define scale 31.8               //Scale for gyroscope (From datasheet)
#define MPU_addr 0x68
int reject_count = 0, running_average = 0;
double kc1, kc2, tau = 0.3471;
int samp_period = 5000;

struct motor {
  int duty, pwm_pin, enable, IMU_ENABLE;
  byte speed_pin;
  float kt, mean, spw, spf, angle, angle_last, angle_speed, ang_err, curr, k1, k2, k3;
  int sam_start, sam_slut, timer_var, time_now, time_last;
  float  ANGLE_REF, cycle_speed[81];
  double acc_angle_1[2], gyro_angle_1[2], comp_angle_1[2], AcX, AcY, GyZ, az, z;
  int freq_max, freq_min, freq_mid,cycle, reject_count,running_average;
  bool add_cycle;
};


#if defined (__AVR_ATmega328P__)  // if running arduino uno, pinout
char BOARD[]      = {"UNO"};
motor M[3] = {
  {128, 11, 7, 1, A1, 0.0335, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2, 3, 0, 0, 0, 0, 0, 0.0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 90, 10, 50,0,0,false},
  {128, 10, 12, 2, A3, 0.0335, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2, 3, 0, 0, 0, 0, 0, 0.0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 90, 10, 50,0,0,false},
  {128, 9, 8, 4, A2, 0.0335, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2, 3, 0, 0, 0, 0, 0, 0.0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 90, 10, 50,0,0,false}
};
#elif defined (ARDUINO_SAMD_MKRVIDOR4000)  // if running mkr vidor, pinout
char BOARD[]      = {"SAMD"};
motor M[3] = {
  {128, 3, 6, 0, A4, 0.0335, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2, 3, 0, 0, 0, 0, 0, 0.0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 90, 10, 50,0,0,false},
  {128, 4, 8, 1, A6, 0.0335, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2, 3, 0, 0, 0, 0, 0, 0.0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 90, 10, 50,0,0,false},
  {128, 5, 7, 2, A5, 0.0335, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 2, 3, 0, 0, 0, 0, 0, 0.0, {}, {}, {}, {}, 0, 0, 0, 0, 0, 90, 10, 50,0,0,false}
};
#endif
int motno;
#define repeat(x) for(motno=0;motno<3;motno++){x;}
#define motors M[motno]


void setup() {
  //  Serial.begin(9600);
  for (int a = 0; a < CHECK_CYCLE; a++)  repeat(motors.cycle_speed[a] = 0.0);
  repeat(pinMode(motors.IMU_ENABLE, OUTPUT))
  repeat(digitalWrite(motors.IMU_ENABLE, LOW))
  repeat(pinMode(motors.enable, OUTPUT))
  repeat(pinMode(motors.pwm_pin, OUTPUT))
  repeat(motors.time_last = millis())
  repeat(motors.sam_start = micros())
  repeat(motors.sam_slut = micros())
  repeat(analogWrite(motors.pwm_pin, motors.duty)) //pwm to 50 % (128)
  repeat(
    Wire.begin();
    digitalWrite(motors.IMU_ENABLE, HIGH);
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); Wire.write(0);
    Wire.endTransmission(true);
    //Gyro config
    Wire.beginTransmission(MPU_addr);       //Contact IMU for setup
    Wire.write(0x1B);                       //GYRO_CONFIG register
    Wire.write(0x03);                       //Register bits set to b'00010000 (1000dps full scale)
    Wire.endTransmission(true);             //End transmission for gyro
    //Acc config
    Wire.beginTransmission(MPU_addr);       //Contact IMU for setup
    Wire.write(0x1C);                       //ACCEL_CONFIG register
    Wire.write(0b00000000);                 //full scale of the IMU
    Wire.endTransmission(true);
    digitalWrite(motors.IMU_ENABLE, LOW))
    filter_setup();
}
float interpolate(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void GetIMUData() {
  repeat(
    int i = 1;
    motors.z = 0;
    Wire.begin();
    digitalWrite(motors.IMU_ENABLE, HIGH);
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); //Specify address for accelerometer
    Wire.endTransmission(true);
    Wire.requestFrom(MPU_addr, 4, true);
  if (Wire.available()) {
  motors.AcX = Wire.read() << 8 | Wire.read();
    motors.AcY = Wire.read() << 8 | Wire.read();
  }
  if (!Wire.available()) Wire.end();
  motors.AcX = (motors.AcX / 16384.0) * 9.81;
  motors.AcY = (motors.AcY / 16384.0) * 9.81;
  if (motors.AcX > 1870.0)  motors.AcX = motors.AcX - 4000.0;
  if (motors.AcY > 1870.0)  motors.AcY = motors.AcY - 4000.0;
  if (abs(motors.AcY) < 0.1 || abs(motors.AcX) < 0.1)  GetIMUData();
  motors.az = -(((atan2(-motors.AcY, -motors.AcX) + PI) * RAD_TO_DEG) - 45);
        while (i <= motors.mean) {
          Wire.begin();
            Wire.beginTransmission(MPU_addr);
            Wire.write(0x47);   //Specify address for gyroscope
            Wire.endTransmission(true);
            Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
            if (Wire.available())  motors.GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
            if (!Wire.available()) Wire.end();
            if (motors.GyZ > 32767)  motors.GyZ = motors.GyZ - 65535;  //Used to wrap values from max around 0
            motors.GyZ = motors.GyZ / scale;
            if (motors.GyZ > 500) motors.GyZ = 500;
            motors.z += motors.GyZ;
            i++;
          }
  motors.GyZ = (motors.z / motors.mean);
               digitalWrite(motors.IMU_ENABLE, LOW))
}
void filter_setup() {
  GetIMUData();
  kc1 = ( 2 * tau - (samp_period / 1000000.0)) / (2 * tau + (samp_period / 1000000.0)); //Complementary filter
  kc2 = (samp_period / 1000000.0) / (2 * tau + (samp_period / 1000000.0));              //Complementary filter
  repeat(
  motors.acc_angle_1[0] = motors.az;
  motors.acc_angle_1[1] = 0;
  motors.gyro_angle_1[0] = motors.GyZ;
  motors.gyro_angle_1[1] = motors.GyZ;
  motors.comp_angle_1[0] = motors.az;
  motors.comp_angle_1[1] = 0)
}

void complementary() {
  repeat(
  kc1 = ( 2 * tau - ((motors.sam_slut - motors.sam_start) / 1000000.0)) / (2 * tau + ((motors.sam_slut - motors.sam_start) / 1000000.0));
  kc2 = ((motors.sam_slut - motors.sam_start) / 1000000.0) / (2 * tau + ((motors.sam_slut - motors.sam_start) / 1000000.0))
  )
  GetIMUData();
  repeat(
  // Set old measurement data
  motors.acc_angle_1[1] = motors.acc_angle_1[0];
  motors.gyro_angle_1[1] = motors.gyro_angle_1[0];
  motors.gyro_angle_1[0] = motors.GyZ;
  motors.comp_angle_1[1] = motors.comp_angle_1[0];
  motors.acc_angle_1[0] = motors.az;
  motors.comp_angle_1[0] = kc1 * motors.comp_angle_1[1] + kc2 * (motors.acc_angle_1[0] + motors.acc_angle_1[1] + tau * motors.gyro_angle_1[0] + tau * motors.gyro_angle_1[1]) - 3.0 * 0.0174532925 )
}


void angle_pot() {
  complementary();
  repeat(
  motors.angle = (motors.comp_angle_1[0] + motors.ANGLE_REF);
  if (motors.angle > 45.0)motors.angle = 45.0;    //move to +- 45 degrees
  if (motors.angle < -45.0)motors.angle = -45.0;
  motors.ang_err=motors.angle*0.0174532925 )
}
void speed_frame() {
  repeat(motors.spw = (motors.GyZ * 0.0174532925))  //Gyroscope is speed of frame
}

void balancePoint() { // Change ANGLE_REF if we are at constant wheel velocity
  repeat(
  if (motors.cycle == CHECK_CYCLE) { // if we have enough readings
    for (int x = 0; x < CHECK_CYCLE; x++)   motors.running_average = motors.running_average + motors.cycle_speed[x];
    motors.running_average = motors.running_average / CHECK_CYCLE;

    for (int b = 0; b < CHECK_CYCLE; b++)
    { // check if the readings is far away from the average, then we dont change anything
      if (abs(motors.cycle_speed[b]) < abs(motors.running_average) * 0.8 || abs(motors.cycle_speed[b]) > abs(motors.running_average) * 1.2)   motors.reject_count++;
      if (motors.reject_count == reject)  break;
    }
    if (motors.reject_count != reject && abs(motors.running_average) <= 100 * rpm2rad ) { // small step, close to equilibrium
      if (motors.running_average < -step_size)  motors.ANGLE_REF = motors.ANGLE_REF - step_size;
      if (motors.running_average > step_size)   motors.ANGLE_REF = motors.ANGLE_REF + step_size;
    }
    if (reject_count != reject && abs(running_average) > 100 * rpm2rad) { // large step, far away from equilibrium
      if (running_average < -step_size)  motors.ANGLE_REF = motors.ANGLE_REF - large_step_size; //left of eq
      if (running_average > step_size)   motors.ANGLE_REF = motors.ANGLE_REF + large_step_size; //right of eq
    }
    motors.running_average = 0; //reset values
    motors.reject_count = 0;
    motors.cycle = 0;
  }
  motors.cycle=motors.cycle)
}
void updateMotor() {
  repeat(
  motors.sam_slut = micros(); // meassure time
  motors.timer_var = micros() )
  angle_pot();           // calculate angle error
  speed_frame();         // calculate frame speed
  repeat(motors.spw = (((float)(((float)analogRead(motors.speed_pin) - 512)) * (2048.0 / 1024.0)) * rpm2rad)) // measure flywheel speed
  repeat(
  motors.curr = (((motors.k1 * motors.spw + motors.k2 * motors.ang_err + motors.k3 * motors.spf)) / motors.kt);  // IMU controller
  if (motors.curr >= CURRENT_MAX)  motors.curr = CURRENT_MAX;  // if above max current set equal to max
  if (motors.curr <= -CURRENT_MAX)  motors.curr = -CURRENT_MAX; // if below min current set equal to min
  motors.duty = (int)interpolate(motors.curr, -CURRENT_MAX, CURRENT_MAX, motors.freq_max, motors.freq_min); // map the current from max to min
//  analogwrite...
  repeat(analogWrite(motors.pwm_pin, map(motors.duty,0,100,0,255)))
  if (!motors.add_cycle) motors.cycle_speed[motors.cycle] = motors.spw; // save the speed for the ANGLE_REF correction
  if (!motors.add_cycle) motors.cycle++; // add one to the array
  motors.sam_start = motors.timer_var)
}

void loop() {
  repeat(
  digitalWrite(motors.enable, HIGH))
  if (micros() - motors.timer_var >= samp_period) updateMotor(); // if we have waited the sampling time.
  balancePoint();  // check if ANGLE_REF needs correction
}
