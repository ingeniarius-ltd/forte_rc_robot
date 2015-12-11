/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Ingeniarius,Ltd.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the company Ingeniarius, Ltd nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Version: 1.0
* Last change: 11/12/2015
*********************************************************************/

/************************************************************************
 *
 * Load libraries
 *
 *************************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt16MultiArray.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <TimerOne.h>
#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>
#include <AdafruitNeoPixel.h>
#include <EEPROM.h>
#include "Robot.h"
#include "RobotSerialComm.h"
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(2, 13, NEO_GRB + NEO_KHZ800); //2 Status LED // PIN13


// RobotSerial Communication
RobotSerialComm port;


ros::NodeHandle  nh;
ros::Time current_time, last_time;

//publish tf
tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped t;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::Twist message_cmd_vel;
std_msgs::UInt16MultiArray envio;
ros::Publisher sonars("sonars", &envio);

//publish odom
nav_msgs::Odometry odometria_envio;
ros::Publisher odom("odom", &odometria_envio);



/************************************************************************
 *
 * Define global variables
 *
 *************************************************************************/
char base_link[] = "/base_link";

//encoder count - 12750 -> 1 turn     19972 -> 1 meter 
Encoder encRF(2, 4), encRB(18, 17), encLF(19, 16), encLB(3, 5);
long int encRF_old = 0, encRB_old = 0, encLF_old = 0, encLB_old = 0, encRF_new = 0, encRB_new = 0, encLF_new = 0, encLB_new = 0;
long int encRF_oldvel = 0, encRB_oldvel = 0, encLF_oldvel = 0, encLB_oldvel = 0, encRF_newvel = 0, encRB_newvel = 0, encLF_newvel = 0, encLB_newvel = 0;

uint16_t sonararray[16] = {
  0};
int sonarID[16] = {
  112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127};
int sonarIN = 0;
uint16_t sonarReadings[16] = {
  0};

// create servo objects to control the four motors (R - right; L - left; F - front; B - back)
#define Epin  12
#define motorRFpin 8
#define motorRBpin 9
#define motorLFpin 10
#define motorLBpin 11
Servo motorRF, motorRB, motorLF, motorLB;

// define controller gains for the motors, as well as errors
float Kp = -0.05, Kd = 0.2, Ki = 0.002;
float EpRF = 0.0, EpRF_old = 0.0, EdRF = 0.0, EiRF = 0.0;
float EpRB = 0.0, EpRB_old = 0.0, EdRB = 0.0, EiRB = 0.0;
float EpLF = 0.0, EpLF_old = 0.0, EdLF = 0.0, EiLF = 0.0;
float EpLB = 0.0, EpLB_old = 0.0, EdLB = 0.0, EiLB = 0.0;

// robot phisical parameters - meters
float m_dDiam = 0.2032, m_dAxis1Length = 0.5, m_dAxis2Length = 0.5;

float positionx = 0, positiony=0, positionw=0;
float velocities[4] = {
  0};
int VelRF_real = 0, VelRB_real = 0, VelLF_real = 0, VelLB_real = 0;

long int startDelay = 0;

byte encoder_readings[8] = {255};
//float Vx=0, Vy=0, Vw=0;

//flag for streaming
boolean flagStream = false;


//SPEEDs -> Actualizar com valores subscriço
//float linearx = 0; //  m/s     >0 front   <0 backwards
//float lineary = 0; //  m/s  >0 left    <0 right
//float angularz = 0.0;  //  rad/s >0 rotate left    <0 rotate left


/************************************************************************
 *
 * Function:  STATUS LED.
 * Objective:
 * Issues:    None to report so far.
 *
 *************************************************************************/
void statusLED(int R, int G, int B, int R2, int G2, int B2) {
  pixels.setPixelColor(0, pixels.Color(R, G, B)); // RED / GREEN / BLUE
  pixels.show(); // This sends the updated pixel color to the hardware.
  pixels.setPixelColor(1, pixels.Color(R2, G2, B2)); // RED / GREEN / BLUE
  pixels.show(); // This sends the updated pixel color to the hardware.
}


/************************************************************************
 *
 * Function:  remap
 * Objective: remap velocities to motor input
 * Issues:    None to report so far.
 *
 *************************************************************************/
int remap_num(float vel) {
  vel = vel * 100;
  int pulse = map(vel, -5000, 5000, 0, 18000);

  return pulse / 100;
}


/************************************************************************
 *
 * Function:  Initialize motors (stop position).
 * Objective: Reset encoders and activate the motors
 * Issues:    None to report so far.
 *
 *************************************************************************/
void stopmotors() {
  // initialize motors in the stop position (90 - stop; 0 - high speed reverse; 180 - high speed foward)
  motorRF.write(90);
  motorRB.write(90);
  motorLF.write(90);
  motorLB.write(90);
}

/************************************************************************
 *
 * Function:  encoders readings.
 * Objective: Stop motors and turn on warning led
 * Issues:    None to report so far.
 *
 *************************************************************************/
int encoder_read() {
  encRF_old = encRF.read();
  encRB_old = encRB.read();
  encLF_old = encLF.read();
  encLB_old = encLB.read();

  Serial3.print("@");
  Serial3.print(encRF.read());
  Serial3.print(",");
  Serial3.print(encRB.read());
  Serial3.print(",");
  Serial3.print(encLF.read());
  Serial3.print(",");
  Serial3.print(encLB.read());
  Serial3.println("e");
}

/************************************************************************
 *
 * Function:  Reset encoder count
 * Objective:
 * Issues:    None to report so far.
 *
 *************************************************************************/
int encoders_reset() {
  encRF.write(0);
  encRB.write(0);
  encLF.write(0);
  encLB.write(0);
}

/************************************************************************
 *
 * Function:  Encoder test.
 * Objective: Each wheel turns 2 times at the same speed. Time is calculated and presented
 * Issues:    None to report so far.
 *
 *************************************************************************/
int testEncoders() {
  long int pulsosRF = 0, pulsosLF = 0, pulsosRB = 0, pulsosLB = 0;
  long int t1 = 0, t2 = 0, t3 = 0, t4 = 0;
  int tempoRF = 0, tempoLF = 0, tempoRB = 0, tempoLB = 0;

  t1 = millis();
  while (pulsosRF < 25500) {
    motorRF.write(97);
    pulsosRF = encRF.read();
  }
  motorRF.write(90);
  tempoRF = millis();

  t2 = millis();
  while (pulsosLF < 25500) {
    motorLF.write(97);
    pulsosLF = encLF.read();
  }
  motorLF.write(90);
  tempoLF = millis();


  t3 = millis();
  while (pulsosRB < 25500) {
    motorRB.write(97);
    pulsosRB = encRB.read();
  }
  motorRB.write(90);
  tempoRB = millis();


  t4 = millis();
  while (pulsosLB < 25500) {
    motorLB.write(97);
    pulsosLB = encLB.read();
  }
  motorLB.write(90);
  tempoLB = millis();

  Serial3.print('@');
  Serial3.print (tempoRF - t1);
  
  Serial3.print(',');
  Serial3.print (tempoLF - t2);
 
  Serial3.print(',');
  Serial3.print (tempoRB - t3);

  Serial3.print(',');
  Serial3.print (tempoLB - t4);
  Serial3.println(' e');
  stopmotors();
}

/************************************************************************
 *
 * Function:  readSonar.
 * Objective: Read X(index) sonar
 * Issues:    None to report so far.
 *
 *************************************************************************/
int readSonar(int index) {
  int reading;
  unsigned long timeout = millis();
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(index); 

  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in centimeters
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  delay(70);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(index); 
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(index, 2);    // request 2 bytes from slave device #index

  // step 5: receive reading from sensor
  if (2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}


//timer to refresh sonars
unsigned long timeToSend = millis();
/************************************************************************
 *
 * Function:  sonarfill()
 * Objective: Read X(index) sonar
 * Issues:    None to report so far.
 *
 *************************************************************************/
void sonarfill() {
  // copy sonar data
  for (int i = 0; i < 16; i++)
    sonararray[i] = sonarReadings[i];
  //1 second interval
  if ((millis() - timeToSend) > 1000) {
    timeToSend = millis();
  }
}

/************************************************************************
 *
 * Function:  readAllSonars.
 * Objective: Reads set (4) of sonars (FRONT, LEFT, BACK, RIGHT or ALL)
 * Issues:    None to report so far.
 *
 *************************************************************************/
int readAllSonars(int p) {
  int tab[4][5] = {     0       };
  int ID [4][5] = {     0       };
  int val = 112;

  for (int x = 0; x < 4; x ++) {
    for (int y = 0; y < 5; y ++) {
      ID[x][y] = val;
      val++;
      //Serial.println(ID[x][y]);
    }
    val--;
  }
  ID[3][4] = 112;
  if (p > 0 && p < 5) {
    Serial3.print('@');
    for (int i = 0; i < 4; i++) {
      tab[p - 1][i] = readSonar(ID[p - 1][i]);
      Serial3.print(tab[p - 1][i]);
      Serial3.print(',');
      }
      for (int i = 3; i < 4; i++) {
        Serial3.print(tab[p - 1][i]);
        Serial3.println('e');
      }
    
  }

  if (p==5){
    Serial3.print('@');
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4 ; k++) {
        tab[j][k] = readSonar(ID[j][k]);
        if (ID[j][k]!=127){
          Serial3.print(tab[j][k]);
          Serial3.print(',');
        }
        else{     
          Serial3.print(tab[j][k]);
          Serial3.println('e');
        }
      }  
    }
  }
}

/************************************************************************
 *
 * Function:  mecanumcinematics
 * Objective: define velocities to the 4 engines
 * Issues:    None to report so far.
 *
 *************************************************************************/
void mecanumcinematics(float linearx, float lineary, float angularz) {
  /*
   robot:  l1: m_dAxis1Length
   l2: m_dAxis2Length
   
   --|##2##|        |##4##|
   ^   ##################
   l1  ##################             ^ y
   ¦   ##################             ¦
   v   ##################       x     ¦
   --|##1##|        |##3##|     <-----¦-
   |              |
   |<---- l2 ---->|
   */

  //wheel Front Left
  velocities[0] = 2 / m_dDiam * ( linearx - lineary - (m_dAxis1Length + m_dAxis2Length) / 2 * angularz);
  //wheel Front Right
  velocities[1] = 2 / m_dDiam * ( linearx + lineary + (m_dAxis1Length + m_dAxis2Length) / 2 * angularz);
  //wheel Back Left
  velocities[2] = 2 / m_dDiam * ( linearx + lineary - (m_dAxis1Length + m_dAxis2Length) / 2 * angularz);
  //wheel Back Right
  velocities[3] = 2 / m_dDiam * ( linearx - lineary + (m_dAxis1Length + m_dAxis2Length) / 2 * angularz);
}

/************************************************************************
 *
 * Function:  move_noPID
 * Objective: robot moves 2 meters
 * Issues:    None to report so far.
 *
 *************************************************************************/
int move_noPID() {
  int distDone = 0;
  int distToDo = 39944;

  while (distDone < distToDo) {
    distDone = (int)((float)(((encRF.read() + encRB.read() + encLF.read() + encLB.read()) / 4) * 2 * PI * 20.32) / 3000) ;
    // start moving - This is only here because we intend to implement a speed controller at some point!
    motorRF.write(98);
    motorRB.write(98);
    motorLF.write(98);
    motorLB.write(98);
    encoder_read();
  }
  stopmotors();
}


/************************************************************************
 *
 * Function:  move_PID
 * Objective: robot moves 2 meters - PID ASSISTED
 * Issues:    None to report so far.
 *
 *************************************************************************/
long int dif_T=0;
long int pulses = 39944;
int move_PID(float linearx, float lineary, float angularz) {
  // check position
  mecanumcinematics(linearx, lineary, angularz);
  //if ((abs(encRF.read()) + abs(encRB.read()) + abs(encLF.read()) + abs(encLB.read())) / 4 < pulses) {

  //remap velocities to servo motor
  int VelRF_des = remap_num(velocities[1]);
  int VelRB_des = remap_num(velocities[3]);
  int VelLF_des = remap_num(velocities[0]);
  int VelLB_des = remap_num(velocities[2]);

  encRF_new = encRF.read();
  encRB_new = encRB.read();
  encLF_new = encLF.read();
  encLB_new = encLB.read();

  //calculate encoder difference
  long int encRF_diff = encRF_old - encRF_new;
  long int encRB_diff = encRB_old - encRB_new;
  long int encLF_diff = encLF_old - encLF_new;
  long int encLB_diff = encLB_old - encLB_new;

  //calculate time difference
  dif_T=millis()-dif_T;

  //calculate real speed with encoder reading & time
  int VelRF_real = (int) (90 - (float)encRF_diff / (float)(dif_T));
  int VelRB_real = (int) (90 - (float)encRB_diff / (float)(dif_T));
  int VelLF_real = (int) (90 - (float)encLF_diff / (float)(dif_T));
  int VelLB_real = (int) (90 - (float)encLB_diff / (float)(dif_T));

  //PID aplication
  EpRF = (float) (VelRF_des - VelRF_real);
  EdRF = (EpRF - EpRF_old)/(float)(dif_T);
  EiRF = EiRF + EpRF*(float)(dif_T);
  EpRF_old = EpRF;
  int VelRF_add = 90 + (int) (Kp * EdRF + Kd * EdRF + Ki * EiRF);

  EpRB = (float) (VelRB_des - VelRB_real);
  EdRB = (EpRB - EpRB_old)/(float)(dif_T);
  EiRB = EiRB + (float)(dif_T)*EpRB;
  EpRB_old = EpRB;
  int VelRB_add = 90 + (int) (Kp * EdRB + Kd * EdRB + Ki * EiRB);

  EpLF = (float) (VelLF_des - VelLF_real);
  EdLF = (EpLF - EpLF_old)/(float)(dif_T);
  EiLF = EiLF + (float)(dif_T)*EpLF;
  EpLF_old = EpLF;
  int VelLF_add = 90 + (int) (Kp * EdLF + Kd * EdLF + Ki * EiLF);

  EpLB = (float) (VelLB_des - VelLB_real);
  EdLB = (EpLB - EpLB_old)/(float)(dif_T);
  EiLB = EiLB + (float)(dif_T)*EpLB;
  EpLB_old = EpLB;
  int VelLB_add = 90 + (int) (Kp * EdLB + Kd * EdLB + Ki * EiLB);

  encRF_old = encRF_new;
  encRB_old = encRB_new;
  encLF_old = encLF_new;
  encLB_old = encLB_new;

  dif_T=millis();

  if ((VelRF_add > 60) && (VelRF_add < 120))
    motorRF.write(VelRF_add);
  if ((VelRB_add > 60) && (VelRB_add < 120))
    motorRB.write(VelRB_add);
  if ((VelLF_add > 60) && (VelLF_add < 120))
    motorLF.write(VelLF_add);
  if ((VelLB_add > 60) && (VelLB_add < 120))
    motorLB.write(VelLB_add);

}

//PID vars
float Vx=0, Vy=0, Vw=0;
/************************************************************************
 *
 * Function:  PING PONG.
 * Objective: send command, get action
 * Issues:    None to report so far.
 *
 *************************************************************************/
void pingpong() {

  unsigned int arg[5];
  int action = port.getMsg(arg);
  switch (action) {

  case ACTION_START_STREAM:             //@1e, no reply
    flagStream = true;
    break;

  case ACTION_STOP_STREAM:             //@2e, no reply
    flagStream = false;
    break;

  case ENCODER_TEST:                    //@3e, reply: @3<LFsec><RFsec><LBsec><RBsec>e
    testEncoders();
    break;

  case READ_ENCODERS:                    //@4e, reply: @4<LF><RF><LB><RB>e
    encoder_read();
    break;

  case READ_SONARS_FRONT:                //@5e, reply: @5<S1><S2><S3><S4><S5>e
    readAllSonars(1);
    break;

  case READ_SONARS_RIGHT:                //@6e, reply: @6<S5><S6><S7><S8><S9>e
    readAllSonars(2);
    break;

  case READ_SONARS_BACK:                //@7e, reply: @7<S9><S10><S11><S12><S13>e
    readAllSonars(3);
    break;

  case READ_SONARS_LEFT:                //@8e, reply: @8<S13><S14><S15><S16><S1>e
    readAllSonars(4);
    break;

  case READ_SONARS_ALL:                 //@9e, reply: @9<S1><S2><S3><S4><S5><S6><S7><S8><S9><S10><S11><S12><S13><S14><S15><S16>e
    readAllSonars(5);
    break;

  case MOVE_PID:                        //@10e, no reply
    move_PID(Vx, Vy, Vw);
    break;

  case MOVE_NOPID:                      //@11e, no reply
    move_noPID();
    break;

  case STOP_MOTORS:                     //@12e, no reply
    stopmotors();
    break;

  case ENCODERS_RESET:                  //@13e, no reply
    encoders_reset();
    break;
    
  case LED_STATE:                    //@14,R1,G1,B1,R2,G2,B2,e, no reply
    statusLED(arg[0], arg[1], arg[2], arg[3], arg[4], arg[5]);
    break;  
    
  default:
    break;

  }
}

/************************************************************************
 *
 * Function:  messageCb
 * Objective: callback function to cmd_vel topic
 * Issues:    None to report so far.
 *
 *************************************************************************/
void messageCb ( const geometry_msgs::Twist &incoming_msg ){
  move_PID(incoming_msg.linear.x, incoming_msg.linear.y, incoming_msg.angular.z);
  nh.spinOnce();
}

//subscribe rostopic cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/************************************************************************
 *
 * Function:  Initial setup.
 * Objective: Open Serial ports
 *            Open I2C
 *            Start streaming interrupt
 * advertise/subscribe topics
 * Issues:    None to report so far.
 *
 *************************************************************************/
void setup(){
  //second serial port
  Serial3.begin(115200);

  //emergency stop button
  pinMode(Epin, INPUT); // pin 12

  nh.getHardware()->setBaud(115200);

  nh.initNode();
  broadcaster.init(nh);

  //sonar array length
  envio.data_length = 16;

  //topic's
  nh.advertise(sonars);
  nh.subscribe(sub);
  nh.advertise(odom);


  //start streaming timer
  Timer1.attachInterrupt(sonarfill, 0); // by default, we consider 100ms -> 10Hz
  Timer1.stop();  // check if timer is really not running!!!

  // create i2c bus (sonars)
  Wire.begin();
  
  // initializes the NeoPixel library.
  pixels.begin(); 

  // attach motors to the servo objects
  motorRF.attach(motorRFpin, 1000, 2000); // attaches the right-front motor on pin 8 to the servo object
  motorRB.attach(motorRBpin, 1000, 2000); // attaches the right-back motor on pin 9 to the servo object
  motorLF.attach(motorLFpin, 1000, 2000); // attaches the left-front motor on pin 10 to the servo object
  motorLB.attach(motorLBpin, 1000, 2000); // attaches the left-back motor on pin 11 to the servo object
}

/************************************************************************
 *
 * Function:  Pcart
 * Objective: Determine robot velocities components (Vx Vy Wz)
 * Issues:    None to report so far.
 *
 *************************************************************************/
//Determine robot velocities components 
float velocidades[3] = {
  0};
void Pcart(float vel0, float vel1, float vel2, float vel3) {
  velocidades[0] = (vel0 +  vel1 + vel2 +  vel3) * m_dDiam / 8;  //Vx
  velocidades[1] = (vel1 -  vel0 - vel3 +  vel2) * m_dDiam / 8;  //Vy
  velocidades[2] = (-vel0 + vel1 - vel2 + vel3) * m_dDiam / 4 / (m_dAxis1Length + m_dAxis2Length); //Wz
}



/************************************************************************
 *
 * Function:  Main loop.
 * Objective: 
 * Issues:    None to report so far.
 *
 *************************************************************************/
long int time1;
double last_time_pub = 0;
long int tempo1=millis();
long int tempo2=millis();
long int tempo3=millis();

void loop() {

  //checks Stop button status -> sonarReadings
  if (digitalRead(Epin) == HIGH ) {

    move_PID(linearx, lineary, angularz);
  //SONARs readings for topic sonars
  if (startDelay == 0) { // only move on to another sensor when we're sure we've read the measurement from the current one
      // step 1: instruct sensor to read echoes
      Wire.beginTransmission(sonarID[sonarIN]);
      Wire.write(byte(0x00));
      Wire.write(byte(0x51));
      Wire.endTransmission();
      // if we need to do other things that may need some delay, do them here and use startDelay to do so
      startDelay = millis();  // start counting time for reading to happen
    }

    // receives PING-PONG command
    if (Serial3.available() > 0) 
      pingpong();

    sonarfill();

    // step 2: wait for readings to happen
    if ((millis() - startDelay) > 65) { // datasheet suggests at least 65 milliseconds

      // step 3: instruct sensor to return a particular echo reading
      Wire.beginTransmission(sonarID[sonarIN]); // transmit to device #112
      Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
      Wire.endTransmission();      // stop transmitting

      // step 4: request reading from sensor
      Wire.requestFrom(sonarID[sonarIN], 2);    // request 2 bytes from slave device #112

        // step 5: receive reading from sensor
      if (2 <= Wire.available()) {  // if two bytes were received
        sonarReadings[sonarIN] = Wire.read();  // receive high byte (overwrites previous reading)
        sonarReadings[sonarIN] = sonarReadings[sonarIN] << 8;    // shift high byte to be high 8 bits
        sonarReadings[sonarIN] |= Wire.read(); // receive low byte as lower 8 bits
      }

      // move on to next ssonar
      sonarIN++;
      //soanar index loop
      if (sonarIN >= 16) {
        sonarIN = 0;
      }
      startDelay=0;
    }

    if (flagStream==true){
     // if(millis()-tempo3>100){
        
      //}


      if(millis()-tempo1>100){
        
        tempo1 = millis();
        envio.data = sonararray;
        sonars.publish( &envio);
        nh.spinOnce();
        
        tempo1 = millis();
        //encoder counts -> velocities
        encRF_newvel = encRF.read();
        encRB_newvel = encRB.read();
        encLF_newvel = encLF.read();
        encLB_newvel = encLB.read();



        long int encRF_diffvel = encRF_oldvel - encRF_newvel;
        long int encRB_diffvel = encRB_oldvel - encRB_newvel;
        long int encLF_diffvel = encLF_oldvel - encLF_newvel;
        long int encLB_diffvel = encLB_oldvel - encLB_newvel;

        //real speed calculation
        long int deltat = (micros() - time1);
        time1 = micros();

        //real speed from each wheel
        //Serial.println("################");
        float TspeedRB = ((0 - (float)encRB_diffvel) / 19972.0) / ((float)deltat / 1000000.0) ; //19972 pulses = 1meter
        //Serial.println(TspeedRB);
        float TspeedLB = ((0 - (float)encLB_diffvel) / 19972.0) / ((float)deltat / 1000000.0) ;
        //Serial.println(TspeedLB);
        float TspeedRF = ((0 - (float)encRF_diffvel) / 19972.0) / ((float)deltat / 1000000.0) ;
        //Serial.println(TspeedRF);
        float TspeedLF = ((0 - (float)encLF_diffvel) / 19972.0) / ((float)deltat / 1000000.0) ;
        //Serial.println(TspeedLF);

        //location
        Pcart (TspeedLF, TspeedRF, TspeedLB, TspeedRB);
        positionx = positionx + ( velocidades[0] * ((float)deltat / 100000.0));
        positiony = positiony + ( velocidades[1] * ((float)deltat / 100000.0));
        positionw = positionw + ( velocidades[2] * ((float)deltat / 100000.0));

        //refresh encoder count
        encRF_oldvel = encRF_newvel;
        encRB_oldvel = encRB_newvel;
        encLF_oldvel = encLF_newvel;
        encLB_oldvel = encLB_newvel;

        last_time_pub = current_time.toSec();

        // First, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;

        //odom_trans.header.stamp = current_time;
        odom_trans.header.stamp = nh.now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = positionx;
        odom_trans.transform.translation.y = positiony;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation = tf::createQuaternionFromYaw(positionw);

        // Send the transform
        broadcaster.sendTransform(odom_trans); 		// odom->base_link transform
        nh.spinOnce();

        //////////////////////////////////////-----ODOM-----///////////////////////////////////////////////////////////////////
        // Next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odometria_envio;
        //odometria_envio.header.stamp = current_time;
        odometria_envio.header.stamp = nh.now();
        odometria_envio.header.frame_id = "odom";

        // Set the position
        odometria_envio.pose.pose.position.x = positionx;
        odometria_envio.pose.pose.position.y = positiony;
        odometria_envio.pose.pose.position.z = 0.0;
        odometria_envio.pose.pose.orientation = odom_trans.transform.rotation;

        // Set the velocity
        odometria_envio.child_frame_id = "base_link";
        odometria_envio.twist.twist.linear.x = velocidades[0];
        odometria_envio.twist.twist.linear.y = velocidades[1];
        odometria_envio.twist.twist.angular.z = velocidades[2];


        odom.publish(&odometria_envio);		// odometry publisher
        nh.spinOnce();

      }
    }
  }

  //stop buttos pushed
  else{
    stopmotors();
    encoders_reset();
  }

}







