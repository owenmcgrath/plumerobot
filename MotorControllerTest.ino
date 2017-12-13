/*_____  _                        _____       _           _   
 |  __ \| |                      |  __ \     | |         | |  
 | |__) | |_   _ _ __ ___   ___  | |__) |___ | |__   ___ | |_ 
 |  ___/| | | | | '_ ` _ \ / _ \ |  _  // _ \| '_ \ / _ \| __|
 | |    | | |_| | | | | | |  __/ | | \ \ (_) | |_) | (_) | |_ 
 |_|    |_|\__,_|_| |_| |_|\___| |_|  \_\___/|_.__/ \___/ \__|
 
 A 4-wheeled differential steering robot that measures CO2 concentration

 Authors: Owen McGrath, Ben Preston, Spyros Kasapis, Marios Kontopyrgos, Sly Halama

 EXTERNAL LIBRARIES:

 cozir-master from Github user Rodir at https://github.com/roder/cozir
 adafruit_bno055 from Adafruit at https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 adafruit_sensor from Adafruit at https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 

 FUNCTIONS:

 getImuMeasurements() - Pulls new accelerations and angular velocity from the IMU
 getCO2Measurements() - Pulls new CO2 Concentrations from the Sensors 
 getVel(int pin) - Calculates frequency of the square wave outputted from the motors in either A or B, converts to a velocity
 getVelAverage(int side) - Calculates velocity average for Vl and Vr 
 exitSafeStart(SoftwareSerial mc) - Resets the motor Controllers from "Error Mode" so that the motors can move again after there is an error(red light turns off)
 void setMotion(int type) - Sets motion mode for the robot
 
 */

#include <cozir.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//MATHEMATICAL CONSTANTS AND DIMENSIONS

#define pi 3.141592653589
#define w .25 //robot width
#define MAX_SPEED 3200
String command;
int count = 0;
long motionStartTime = 0;
long currentTime = 0;
int motionStep = 0;
int motionMode = 5;
int timeAmount = 30000;
double omega = 1.338667;
double angle = 0;
int mode = 0;
int isMoving = 0;
int deltaT = 0;
int prevTime = 0;



//ARDUINO INITIALIZATIONS

//arduino pins

//CO2 Sensors
#define co2_f_rx 10
#define co2_f_tx 3
#define co2_b_rx 11
#define co2_b_tx 5
#define co2_l_rx 12
#define co2_l_tx 7
#define co2_r_rx 13
#define co2_r_tx 9

//motor controllers
#define l_controller_rxPin 15 
#define l_controller_txPin 14
#define r_controller_rxPin 17
#define r_controller_txPin 16
 
//encoders
#define rb_encoder_A 35
#define rb_encoder_B 37
#define rf_encoder_A 39
#define rf_encoder_B 41
#define lb_encoder_A 43
#define lb_encoder_B 47
#define lf_encoder_A 49
#define lf_encoder_B 46

//sample rate for determining encoder frequency
#define SAMPLES 16 

//serial connections
SoftwareSerial rmc = SoftwareSerial(r_controller_rxPin, r_controller_txPin);
SoftwareSerial lmc = SoftwareSerial(l_controller_rxPin, l_controller_txPin);
SoftwareSerial co2_f = SoftwareSerial(co2_f_rx, co2_f_tx);
SoftwareSerial co2_b = SoftwareSerial(co2_b_rx, co2_b_tx);
SoftwareSerial co2_r = SoftwareSerial(co2_r_rx, co2_r_tx);
SoftwareSerial co2_l = SoftwareSerial(co2_l_rx, co2_l_tx);

//IMU INITIALIZATIONS
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float z[3] = {0};

//COZIR INITIALIZATIONS
int cf = 0;
int cb = 0;
int cr = 0;
int cl = 0;
int c[4] = {0};





/* FUNCTION: getImuMeasurements()
 * DESCRIPTION:
 * Pulls new accelerations and angular velocity from the IMU
 */
void getImuMeasurements(){
   sensors_event_t event;
   bno.getEvent(&event);
   imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   z[0] = accel.x();
   z[1] = accel.y();
   z[2] = gyro.z();
}

/* FUNCTION: getCO2Measurements()
 * DESCRIPTION:
 * Pulls new CO2 Concentrations from the Sensors
 */
void getCO2Measurements(){
  COZIR czr_f(co2_f);
  co2_f.listen();
  cf = czr_f.CO2();
  
  COZIR czr_r(co2_r);
  co2_r.listen();
  cr = czr_r.CO2();
  
  COZIR czr_b(co2_b);
  co2_b.listen();
  cb = czr_b.CO2();
  
  COZIR czr_l(co2_l);
  co2_l.listen();
  cl = czr_l.CO2();
}

/* FUNCTION: getVel(int pin)
 * DESCRIPTION:
 * Calculates frequency of the square wave outputted from the motors in either A or B, 
 * converts to a velocity 
 * PARAMETERS: 
 * int pin //Pin number for encoder frequency measurement
 * RETURNS:
 * Wheel Velocity (m/s)
 */
float getVel(int pin) {
  long freq = 0;
  for(unsigned int j=0; j<SAMPLES; j++) freq+= 500000/pulseIn(pin, HIGH, 250000);
  return .39*freq/SAMPLES/898;
}

/* FUNCTION: getAppVel(int side)
 * DESCRIPTION:
 * Returns predetermined velocities for a motion mode (see setMotion)
 * PARAMETERS: 
 * int side //0 for Vl, 1 for Vr
 * RETURNS:
 * Wheel Velocity (m/s)
 */
float getAppVel(int side) {
  switch(motionMode){
    case 1: 
      return 0.261;
    case 2: 
      return -0.261;
    case 3:
      if(side == 0){return .184;} else {return -.184;}  
    case 4:
      if(side == 0){return -.184;} else {return .184;} 
    case 5:
      return 0;
  }
}

/* FUNCTION: getVelAverage(int side)
 * DESCRIPTION:
 * Calculates velocity average for Vl and Vr 
 * PARAMETERS: 
 * int side //Left velocity -> 0 or Right Velocity -> 1 
 * RETURNS:
 * Velocity Average for each side both A and B pins.  (m/s)
 */
float getVelAverage(int side){
  //Need this if statement because getVel doesn't work if the motors are not moving, it pauses for a while
  if(isMoving == 0){
    return 0;
  } else {
    switch(side){
      case 0:
        return .25*(getVel(lb_encoder_A)+getVel(lb_encoder_B)+getVel(lf_encoder_A)+getVel(lf_encoder_B));
      case 1:
        return .25*(getVel(rb_encoder_A)+getVel(rb_encoder_B)+getVel(rf_encoder_A)+getVel(rf_encoder_B));  
    }
  }
}

/* FUNCTION: exitSafeStart(Software Serial mc)
 * DESCRIPTION:
 * Resets the motor Controllers from "Error Mode" so that the motors can move again 
 * after there is an error(red light turns off)
 * PARAMETERS: 
 * SoftwareSerial mc //Serial Connection to the motor Controller specified
 */
void exitSafeStart(SoftwareSerial mc){
  mc.write(0x83);
}



/* FUNCTION: setMotorSpeed(int speed, SoftwareSerial mc)
 * DESCRIPTION:
 * Writes Speed command to the Motor controllers over serial
 * PARAMETERS: 
 * int speed //Needs to be in the range -3200 to 3200
 * SoftwareSerial mc //Serial Connection to the Motor Controller specified
 */
void setMotorSpeed(int speed, SoftwareSerial mc){
  if (speed < 0){
    mc.write(0x86); // motor reverse command
    speed = -speed; // make speed positive
  }else{
    mc.write(0x85); // motor forward command
  }
  mc.write(speed & 0x1F);
  mc.write(speed >> 5);
}


/* FUNCTION: setMotion(int type, int time)
 * DESCRIPTION:
 * Sets motion mode for the robot
 * PARAMETERS: 
 * int type //trajectory type for the robot
 *     MOVE FORWARD ->1
 *     MOVE IN REVERSE ->2
 *     CW ROTATION ->3
 *     CCW ROTATION ->4
 *     STOP MOTION ->5 
 */
void setMotion(int type){
  switch(type){
    case 1:
      motionMode = 1;
      isMoving = 1;
      setMotorSpeed(.252*MAX_SPEED, lmc);
      setMotorSpeed(-.2538*MAX_SPEED, rmc);
      break;
    case 2:
      motionMode = 2;
      isMoving = 1;
      setMotorSpeed(-.245*MAX_SPEED, lmc);
      setMotorSpeed(.259*MAX_SPEED, rmc);
      break;
    case 3:
      motionMode = 3;
      isMoving = 1;
      setMotorSpeed(-.245*MAX_SPEED, lmc);
      setMotorSpeed(-.2538*MAX_SPEED, rmc);
      break;
    case 4:
      motionMode = 4;
      isMoving = 1;
      setMotorSpeed(.255*MAX_SPEED, lmc);
      setMotorSpeed(.262*MAX_SPEED, rmc);
      break;
    case 5:
      motionMode = 5;
      isMoving = 0;
      setMotorSpeed(0, lmc);
      setMotorSpeed(0, rmc);
      break;
  }
}


void setup(){
  Serial.begin(9600);
  Serial.println("Setup beginning");

  //initializations for the IMU
  bno.begin();
  bno.setExtCrystalUse(true);
  
  
  // initialize software serial object with baud rate of 19.2 kbps
  rmc.begin(19200);
  lmc.begin(19200);
  // the Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms
  delay(5);
  // if the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate
  rmc.write(0xAA); // send baud-indicator byte
  lmc.write(0xAA); // send baud-indicator byte
  // next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the Motor run
  exitSafeStart(rmc); // clear the safe-start violation and let the Motor run
  exitSafeStart(lmc); // clear the safe-start violation and let the Motor run

  //START AFTER BEING POWERED ON FOR 15 SEC
  motionStartTime = millis();
  Serial.println("Setup Complete");
}

void loop(){
      delay(1000);
      getCO2Measurements();
      c[0] = cf;
      c[1] = cr;
      c[2] = cb;
      c[3] = cl;
      Serial.print("F: "); Serial.print(cf); Serial.print(", ");
      Serial.print("R: "); Serial.print(cr); Serial.print(", ");
      Serial.print("B: "); Serial.print(cb); Serial.print(", ");
      Serial.print("L: "); Serial.println(cl);

//  Serial.print(getAppVel(1),8);
//  Serial.print(", ");
 // getImuMeasurements();
//  currentTime = millis();
//  deltaT = currentTime - prevTime;
//  prevTime = currentTime;
//  Serial.print(deltaT, 8);
////  PRINT VELOCITIES FROM ENCODERS
//  Serial.print(", ");
//  Serial.print(getVelAverage(0), 8);
//  Serial.print(", ");
//  Serial.println(getVelAverage(1), 8);
//PRINT PREDETERMINED VELOCITIES
//  Serial.print(", ");
//  Serial.print(getAppVel(0),8);
//  Serial.print(", ");
//  Serial.print(getAppVel(1),8);
//  Serial.print(", ");
//  if(z[0] <= .9 && z[0] >= -.5){
//    Serial.print("0.0000000");
//  } else {
//    Serial.print(z[0], 8);
//  }
//  Serial.print(", ");
//  if(z[1] <= .9 && z[1] >= -.5){
//    Serial.print("0.0000000");
//  } else {
//    Serial.print(z[1], 8);
//  }
//  Serial.print(", ");
//  if(z[2] <= .5 && z[2] >= -.5){
//    Serial.print("0.0000000");
//  } else {
//    Serial.print(z[2], 8);
//  }
//  Serial.println(", ");
////CO2 GRADIENT CODE
////  start new motion sequence after time "timeAmount" (in ms)
//  if(currentTime-motionStartTime > timeAmount){
//    if(mode == 0){
//      //stop for a delay, get the C02 measurements then display them
//      setMotion(5);
//      delay(10000);
//      getCO2Measurements();
//      c[0] = cf;
//      c[1] = cr;
//      c[2] = cb;
//      c[3] = cl;
//      Serial.print("F: "); Serial.print(cf); Serial.print(", ");
//      Serial.print("R: "); Serial.print(cr); Serial.print(", ");
//      Serial.print("B: "); Serial.print(cb); Serial.print(", ");
//      Serial.print("L: "); Serial.println(cl);
//      //Determine the greatest concentration in front/back then left/right. 
//      int cmax_idx_lr;
//      int cmax_idx_fb;
//      if(c[1] > c[3]){
//        cmax_idx_lr = 1; //right
//      } else if(c[3] > c[1]){
//        cmax_idx_lr = 3; //left
//      } else {
//        return;
//      }
//
//      if(c[0] > c[2]){
//        cmax_idx_fb = 0; //front
//      } else if(c[2] > c[0]){
//        cmax_idx_fb = 2; //back
//      } else {
//        return;
//      }
//      //calculate the gradient between front/back, left/right then find the angle of rotation
//      if(cmax_idx_fb == 0){
//         if(cmax_idx_lr == 1){
//            angle = atan2(c[1]-c[3], c[0]-c[2]);
//         }
//         else{
//            angle = atan2(c[3]-c[1], c[0]-c[2]);
//         }
//         
//      } else {
//         if(cmax_idx_lr == 1){
//            angle = pi - atan2(c[1]-c[3], c[2]-c[0]);
//         }
//         else{
//            angle = pi - atan2(c[3]-c[1], c[2]-c[0]);
//         }
//      }
//      //Calculate the time it needs to rotate at angular velocity omega
//      timeAmount = (angle/omega)*1000;
//      Serial.print("Rotating at angle: "); Serial.print(angle);Serial.print(", for time: "); Serial.println(timeAmount);
//      
//      if(cmax_idx_lr == 1){
//        Serial.print("Moving Clockwise, concentration is: ");
//        Serial.println(c[cmax_idx_lr]);
//        setMotion(3);
//      } else {
//        Serial.print("Moving Counter Clockwise, concentration is: ");
//        Serial.println(c[cmax_idx_lr]);
//        setMotion(4);
//      }
//      motionStartTime = millis();
//      mode = 1;
//    } else if(mode == 1){
//      Serial.println("Moving Forward");
//      setMotion(5);
//      delay(1000);
//      setMotion(1);
//      timeAmount = 1000;
//      motionStartTime = millis();
//      mode = 0;
//    }
//  } 
//  if(currentTime > 300000){
//    Serial.println("Ending....");
//    setMotion(5);
//    while(1){}
//    
//  }
////SQUARE FIGURE 8 MOTION
//  if(currentTime-motionStartTime > timeAmount){
//    switch(motionStep){
//      //forward
//      case 0:
//        Serial.println("Moving Forward");
//        setMotion(5);
//        delay(500);
//        setMotion(1);
//        motionStartTime = millis();
//        count++;  
//        if(count == 1){
//          motionStep = 1;                                                                                                                            
//        } else if(count == 3) {
//          motionStep = 2;
//        } else {
//          motionStep = 3;
//        }
//        timeAmount = 3000;
//        break;
//
//      //CW
//      case 1:
//        Serial.println("Moving CW");
//        setMotion(5);
//        delay(1000);
//        setMotion(3);
//        motionStartTime = millis();
//        count++;
//        motionStep = 0;
//        timeAmount = 950;
//        break;
//        
//      //CCW
//      case 2:
//        Serial.println("Moving CCW");
//        setMotion(5);
//        delay(1000);
//        setMotion(4);
//        motionStartTime = millis();
//        count++;
//        motionStep = 0;
//        timeAmount = 1000;
//        break;
//      //stop
//      case 3:
//         Serial.println("Stopping");
//         setMotion(5);
//         while(1){}
//         
//    }
//  }
}
