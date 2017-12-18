# include <Math.h>
# include <Servo.h>
#include "MPU6050.h"
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION==I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 mpu;

float pitchAccSet, rollAccSet;
float pitchGyroSet, rollGyroSet, yawSet;
double pitchAccSetAnt,rollAccSetAnt,yawSetAnt;
double pitchGyroSetAnt,rollGyroSetAnt;

float roll, pitch, yaw;
int ant_pot = 1050;
int pot = 1050;
//pulsos que se enviaran a los motores 
int m1,m2,m3,m4;
// Variables para los PID
float pitchKp1 = 1;
float pitchKi1 = 0.02;
float pitchKd1 = 0.2;

float rollKp1 = 0.68;
float rollKi1 = 0.025;
float rollKd1 = 0.4;

float yawKp = 0.5;
float yawKi = 0.008;
float yawKd = 0.0;

float pitchKp2 = 1.3;
float pitchKi2 = 0.001;
float pitchKd2 = 0.5;

float rollKp2 = 1.0;
float rollKi2 = 0.001;
float rollKd2 = 0.2;

float accPitch_ierror = 0;
float accRoll_ierror = 0;
float gyroRoll_ierror = 0;
float gyroPitch_ierror = 0;
float yaw_ierror = 0;
float accPitch_last_error = 0 ;
float accRoll_last_error = 0;
float gyroPitch_last_error = 0;
float gyroRoll_last_error = 0;
float yaw_last_error = 0;
float pitchGyroAcc, rollGyroAcc;

//Variables para conseguir los datos del sensor MPU
int16_t ax,ay,az,gx,gy,gz;
float rollGyro,pitchGyro,yawGyro;
float accPitch,accRoll,accYaw;
float ciclo = 5;
float tiempoCiclo = ciclo/100000000;

//Variables para controlar la potencia de los motores 
unsigned char comando;


Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  delay(5000);
  // put your setup code here, to run once:
  
  #if I2CDEV_IMPLEMENTATION==I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR=24;
  #elif I2CDEV_IMPLEMENTATION==I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true);
  #endif
  delay(500);
  //Iniciar la comunicacion serial 
  Serial.begin(9600);
  Serial.setTimeout(10);
  delay(500);
  //Iniciando el sensor 
  Serial.println("Inicializando dispositivos I2C");
  mpu.initialize();
  Serial.println("Verificando conexion MPU6050....");
  Serial.println(mpu.testConnection()?"MPU6050 OK":"MPU6050 ERROR!!!!!!");
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);
  mpu.setRate(4);
  //Declaracion de los offset calculado para este MPU en especifico 
  mpu.setXAccelOffset(-2159);
  mpu.setYAccelOffset(-125);
  mpu.setZAccelOffset(1877);
  mpu.setXGyroOffset(-97);
  mpu.setYGyroOffset(54);
  mpu.setZGyroOffset(8);

  // Inicializar los motores 
  delay(2000);
  iniciar_motores();
  delay(2000);
  mover_motores();
  delay(2000);
   
}

void loop() {
  // put your main code here, to run repeatedly:
  datosMPU();

  pitchGyroSet = pitchGyroSetAnt;
  rollGyroSet = rollGyroSetAnt;
  pitchAccSet = pitchAccSetAnt;
  rollAccSet = rollAccSetAnt;
  yawSet = yawSetAnt;
  pot = ant_pot;

  if(Serial.available() >= 1){
    comando = Serial.read();
    if (comando == 'a'){  //Mas altura
      Serial.println("Mas altura");
      pot += 25;
      pot = constrain(pot, 1050, 2000);
      ant_pot = pot;
      pitchAccSetAnt = 0.0;
      rollAccSetAnt = 0.0;
      pitchGyroSetAnt = 0.0;
      rollGyroSetAnt = 0.0;
      yawSetAnt = 0.0;
    }

    if (comando == 'b'){  // Menos altura
      Serial.println("Menos altura"); 
      pot-=50;
      pot = constrain(pot, 1050, 2000);
      ant_pot = pot;
      pitchAccSetAnt = 0.0;
      rollAccSetAnt = 0.0;
      pitchGyroSetAnt = 0.0;
      rollGyroSetAnt = 0.0;
      yawSetAnt = 0.0;
    }

  control();
    
  }
}

void datosMPU(){
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  //De valores raw a grados 
  float accYangle_raw = atan(((ax)/16384.0)/sqrt(pow(((ay)/16384.0),2)+pow(((az)/16384.0),2)))*180/3.1416;
  float accXangle_raw = atan(((ay)/16384.0)/sqrt(pow(((ax)/16384.0),2)+pow(((az)/16384.0),2)))*180/3.1416;

  pitchGyro = ((gx)/16.4);
  rollGyro = ((-1)*(gy)/16.4);
  yawGyro = ((-1)*(gz)/16.4);

  accPitch=(0.1*accXangle_raw)+(0.9*(accPitch+(pitchGyro*tiempoCiclo)));
  accRoll = (0.1*accYangle_raw)+(0.9*(accRoll+(rollGyro*tiempoCiclo)));
}

void control(){
  // Primera linea de PID
  pitchGyroAcc = accel_pitch(accPitch,pitchAccSet,pitchKp1,pitchKi1,pitchKd1);
  rollGyroAcc = accel_roll(accRoll,rollAccSet,rollKp1,rollKi1,rollKd1);
  //Segunda linea de PID 
  roll = gyro_roll(rollGyro,rollGyroAcc,rollKp2,rollKi2,rollKd2);
  pitch = gyro_pitch(pitchGyro,pitchGyroAcc,pitchKp2,pitchKi2,pitchKd2);
  //Unico PID para yaw 
  yaw = yaw_PID(yawGyro, yawSet, yawKp, yawKi, yawKd);
  //Pulsos para los motores 
  m1 = pot + pitch + roll + yaw;
  m2 = pot + pitch - roll - yaw;
  m3 = pot - pitch - roll + yaw;
  m4 = pot - pitch + roll - yaw;
  //Condicionales para evitar el apagado de los motores 
  if (m1<1050) m1 = 1050;
  if (m2<1050) m2 = 1050;
  if (m3<1050) m3 = 1050;
  if (m4<1050) m4 = 1050;
  //Condicionales para el limite de los motores 
  if (m1>2000) m1 = 2000;
  if (m2>2000) m2 = 2000;
  if (m3>2000) m3 = 2000;
  if (m4>2000) m4 = 2000;

  motores(m1, m2, m3, m4);
}
//primer PID de pitch con datos del acelerometro 
float accel_pitch(float error, float setpoint, float Kp, float Ki, float Kd){
  float err = setpoint - error;
  accPitch_ierror += err;
  accPitch_ierror = constrain(accPitch_ierror, -300, 300);
  double derror = error - accPitch_last_error;
  double out = Kp*err-0.04 + accPitch_ierror*Ki - derror*Kd;
  out = constrain(out, -300, 300);
  accPitch_last_error = error;
  return out;
}
//primer PID de roll con datos del acelerometro
float accel_roll(float error, float setpoint, float Kp, float Ki, float Kd){
  float err = setpoint - error;
  accRoll_ierror += err;
  accRoll_ierror = constrain(accRoll_ierror, -300, 300);
  double derror = error - accRoll_last_error;
  double out = Kp*err + 0.02 + accRoll_ierror*Ki - derror*Kd;
  out = constrain(out, -300, 300);
  accRoll_last_error = error;
  return out;
}
//segundo PID de pitch con datos del giroscopio y el primer PID
float gyro_pitch(float error, float setpoint, float Kp, float Ki, float Kd){
  float err = setpoint - error;
  gyroPitch_ierror += err;
  gyroPitch_ierror = constrain(gyroPitch_ierror, -300, 300);
  double derror = error - gyroPitch_last_error;
  double out = Kp*err + gyroPitch_ierror*Ki - derror*Kd;
  out = constrain(out, -300, 300);
  gyroPitch_last_error = error;
  return out;
}
//segundo PID de roll con datos del giroscopio y el primer PID
float gyro_roll(float error, float setpoint, float Kp, float Ki, float Kd){
  float err = setpoint - error;
  gyroRoll_ierror += err;
  gyroRoll_ierror = constrain(gyroRoll_ierror, -300, 300);
  double derror = error - gyroRoll_last_error;
  double out = Kp*err + gyroRoll_ierror*Ki - derror*Kd;
  out = constrain(out, -300, 300);
  gyroRoll_last_error = error;
  return out;
}
//PID de yaw 
float yaw_PID(float error, float setpoint, float Kp, float Ki, float Kd){
  float err = setpoint - error;
  yaw_ierror += err;
  yaw_ierror = constrain(yaw_ierror, -250, 250);
  double derror = error - yaw_last_error;
  double out = Kp*err + 0.15 + yaw_ierror*Ki - derror*Kd;
  out = constrain(out, -250, 250);
  yaw_last_error = error;
  return out;
}

void iniciar_motores(){
  motor1.attach(9);
  motor2.attach(6);
  motor3.attach(5);
  motor4.attach(3);

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}

void mover_motores(){
  //Comenzar el movimiento de los motores, con la potencia mas baja 
  motor1.writeMicroseconds(1050);
  motor2.writeMicroseconds(1050);
  motor3.writeMicroseconds(1050);
  motor4.writeMicroseconds(1050);
}

void motores(int m1, int m2, int m3, int m4){
  /*Serial.print("Motor 1");Serial.println(m1);
  Serial.print("Motor 2");Serial.println(m2);
  Serial.print("Motor 3");Serial.println(m3);
  Serial.print("Motor 4");Serial.println(m4); */ 
  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);
}

