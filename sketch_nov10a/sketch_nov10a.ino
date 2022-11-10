#include "Wire.h"
#define encoderxA 
#define encoderxB 
#define encoderyA 
#define encoderyB 
#define resolution 540.0
#define R
#define pi 3.147
double countsx, dcountx;
double lastcountx = 0;
double countsy, dcounty;
double lastcounty = 0;
int16_t gyrzOffset ;
int16_t gyrz=0;
unsigned long lastTime ;
unsigned long currentTime ;
float scaled;
float yaw;
double x ,y=0;
double xmap,ymap=0;
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
ros::NodeHandle nh;
geometry_msgs::Pose xx,yy;
sensor_msgs::Imu yawAngle;
//ros::Publisher pub;
//pub=nh.ad
void setup() {    

  nh.getHardware()->setPort(&Serial);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  pub=nh.advertise<geometry_msgs::Pose>("mytopic",0);
  // put your setup code here, to run once:
  pinMode(encoderxA, INPUT_PULLUP);
  pinMode(encoderxB, INPUT_PULLUP);
  pinMode(encoderyA, INPUT_PULLUP);
  pinMode(encoderyB, INPUT_PULLUP);
  attachInterrupt(encoderxA, isr1A, CHANGE);
  attachInterrupt(encoderxB, isr1B, CHANGE);
  attachInterrupt(encoderyA, isr2A, CHANGE);
  attachInterrupt(encoderyB, isr2B, CHANGE);

}

void loop() {
    geometry_msgs::Pose Goal;

  currentTime=millis();
  if(currentTime - lastTime >= 10){
    read();
    gyrz -= gyrzOffset;
    scaled = scaling(gyrz);
    yaw += (scaled*0.01); 
    x = (countsx * 2*pi*R)/resolution; // distance moved in  robot x coordinate 
    y = (countsy * 2*pi*R)/resolution; //distance moved in robot y coordinate
    xmap += (y *cos( 90 -yaw ))+(x * cos (yaw));
    ymap += (y * cos (yaw))- (x sin (yaw));
    lastTime = currentTime;
    //Serial.println(yaw);
    Goal.pose.position.x()=xmap;
    Goal.pose.position.y()=ymap;
    Goal.pose.position.x()=0;

    yaw_msg.data = yaw;
   // pub.publish( &yaw_msg );
    nh.spinOnce();
    pub.publish(Goal);
  
  }
  // put your main code here, to run repeatedly:
   
}
void isr1A() {
  if (digitalRead(encoderxA) != digitalRead(encoderxB))
    countsx++;
  else
    countsx--;
}

void isr1B() {
  if (digitalRead(encoderxA) == digitalRead(encoderxB))
    countsx++;
  else
    countsx--;

}
void isr2A() {
  if (digitalRead(encoderyA) != digitalRead(encoderyB))
    countsy++;
  else
    countsy--;
}

void isr2B() {
  if (digitalRead(encoderyA) == digitalRead(encoderyB))
    countsy++;
  else
    countsy--;

}
void initt(){
Wire.beginTransmission(IMU);
Wire.write(PWR);
Wire.write(0x00);
Wire.endTransmission();
}

void gyro_confg(){
Wire.beginTransmission(IMU);
Wire.write(gyro);
Wire.write(0x10); //full scale +/-1000 deg/sec
Wire.endTransmission();
}
void accl_confg(){
Wire.beginTransmission(IMU);
Wire.write(accl);
Wire.write(0x10); //full scale +/- 8g
Wire.endTransmission();
}
void read(){
Wire.beginTransmission(IMU);
Wire.write(gyrz_add);
Wire.endTransmission();
Wire.requestFrom(IMU,2);
while(Wire.available()<2);
gyrz=Wire.read()<<8 | Wire.read();  

}
void calibration(){
  Serial.println("CALIBRATION!!");
  for (int i=0; i<samples;i++){
    read();
    gyrzOffset+=gyrz;
  }
  
  gyrzOffset/=samples;
  
}
float scaling(float value){
float trueV =(value * 1000.0 ) /32786;
return trueV;
}
