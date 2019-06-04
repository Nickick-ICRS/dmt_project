// ROS stuff will only compile if the libraries etc. are installed in the correct locations.

// Commenting this out will result in none of the ROS dependencies being included in the compile

#define USE_ROS

#ifdef USE_ROS

#include <ros.h>

#include <cynaptix/FrameMeasuredData.h>

#include <cynaptix/FrameTarget.h>

#endif // USE_ROS



/*

 * ##################

 * NORMAL DEFINITIONS

 * ##################

 */
 
  //----------------------------------------------------------------------------------------
//THE FOLLOWING PROGRAM CONTROLS THE ENTIRE ROBOT FRAME AND CALCULATES THE TORQUE FROM EACH MOTOR

#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
Servo Rotation; //initialise servo

#define in1X 4 //input pin 1 for X motor set on PWM pin of arduino
#define in2X 5 //input pin 2 for X motor set on digital pin of arduino
#define in1Y 6
#define in2Y 7
#define in1Z 8 
#define in2Z 9
#define in1Grab 10 
#define in2Grab 11
#define inRot 12

#define encod1X 2 //encoder input channel A
#define encod2X 22 //encoder input channel B
#define encod1Y 3
#define encod2Y 24
#define encod1Z 18
#define encod2Z 19
#define encod1Grab 20
#define encod2Grab 21

#define zero_switchX 42 //pullup pin for switch
#define zero_switchY 40
#define zero_switchZ 38
#define zero_switchGrab 36

#define current_pinX A0 //analogue pin for current sensor
#define current_pinY A1
#define current_pinZ A2
#define current_pinGrab A3
#define current_pinRot A4


  //PID controller setup
  double encod_countX; //encoder position in mm
  double encod_countY;
  double encod_countZ;
  double encod_countGrab;
  
  double speed_X; //output //current position is constantly updated
  double speed_Y;
  double speed_Z;
  double speed_Grab;

  double targetX; //setpoint, target will be given
  double targetY;
  double targetZ;
  double targetGrab;
  double targetRot;

  double KpX=4, KiX=2, KdX=0;
  double KpY=6, KiY=3, KdY=0;
  double KpZ=1, KiZ=0, KdZ=0;
  double KpGrab=0.7, KiGrab=0.52, KdGrab=0;

  char switch_statusX; //0 if off, 1 if on
  char switch_statusY;
  char switch_statusZ;
  char switch_statusGrab;

  double currentX = 0; //current calculated from current sensor
  double currentY = 0;
  double currentZ = 0;
  double currentGrab = 0;
  double currentRot = 0;
  
  double torqueX = 0; //torque calculated from current
  double torqueY = 0;
  double torqueZ = 0;
  double torqueGrab = 0;
  double torqueRot = 0;
  
  int VoutX = 0; //voltage output from current sensor
  int VoutY = 0;
  int VoutZ = 0;
  int VoutGrab = 0;
  int VoutRot = 0;

  //PID instance
  PID myPIDX(&encod_countX, &speed_X, &targetX, KpX, KiX, KdX, DIRECT);
  PID myPIDY(&encod_countY, &speed_Y, &targetY, KpY, KiY, KdY, DIRECT);
  PID myPIDZ(&encod_countZ, &speed_Z, &targetZ, KpZ, KiZ, KdZ, DIRECT);
  PID myPIDGrab(&encod_countGrab, &speed_Grab, &targetGrab, KpGrab, KiGrab, KdGrab, DIRECT);

  Encoder myEncX(encod1X,encod2X);
  Encoder myEncY(encod1Y,encod2Y);
  Encoder myEncZ(encod1Z,encod2Z);
  Encoder myEncGrab(encod1Grab,encod2Grab);

  //----------------------------------------------------------------------------------------


/*

 * ###############

 * ROS DEFINITIONS

 * ###############

 */

#ifdef USE_ROS

// Uncomment this to average all readings since the last message when sending data to frame

// Comment this to send only the latest measured position when sending data to the frame

#define AVERAGE_READINGS



// Communication frequency with the server (Hz)

#define PUBLISH_FREQ 10



// Callback for when we receive new callbacks

void frame_target_callback(const cynaptix::FrameTarget& msg);


// Node handle to interface with ROS

ros::NodeHandle nh;



// Data we have measured message

cynaptix::FrameMeasuredData measured_data_msg;



// Target positions message

cynaptix::FrameTarget target_msg;



// Publisher

ros::Publisher pub("frame_measured_data", &measured_data_msg);



// Subscriber

ros::Subscriber<cynaptix::FrameTarget> sub("frame_target", &frame_target_callback);



// Timer variable to control publish rate

unsigned long ros_timer;



// Variable used to count the number of messages since the previous send when averaging

#ifdef AVERAGE_READINGS

int average_counter;

#endif // AVERAGE_READINGS



#endif // USE_ROS



// Function forward declarations

void setup_ros();



double update_x_pos();

double update_y_pos();

double update_z_pos();

double update_yaw_pos();

double update_grab_pos();



double measure_x_trq();

double measure_y_trq();

double measure_z_trq();

double measure_yaw_trq();

double measure_grab_trq();



void update_ros(double x_pos, double y_pos, double z_pos, double yaw_pos, double grab_pos,

                double x_trq, double y_trq, double z_trq, double yaw_trq, double grab_trq);



void setup() {

  // Setup other shit, inc. home axis if we're implementing that

  // Setup ROS - do this last

  setup_ros();

  pinMode(in1X, OUTPUT); //input driver pin set to output
  pinMode(in2X, OUTPUT); //input driver pin set to output
  pinMode(in1Y, OUTPUT);
  pinMode(in2Y, OUTPUT);
  pinMode(in1Z, OUTPUT);
  pinMode(in2Z, OUTPUT);
  pinMode(in1Grab, OUTPUT);
  pinMode(in2Grab, OUTPUT);
  
  pinMode(encod1X, INPUT_PULLUP); //encoder pin set to input
  pinMode(encod2X, INPUT_PULLUP);
  pinMode(encod1Y, INPUT_PULLUP);
  pinMode(encod2Y, INPUT_PULLUP);
  pinMode(encod1Z, INPUT_PULLUP);
  pinMode(encod2Z, INPUT_PULLUP);
  pinMode(encod1Grab, INPUT_PULLUP);
  pinMode(encod2Grab, INPUT_PULLUP);
  
  pinMode(zero_switchX, INPUT_PULLUP); //switch pin set to input
  pinMode(zero_switchY, INPUT_PULLUP);
  pinMode(zero_switchZ, INPUT_PULLUP);
  pinMode(zero_switchGrab, INPUT_PULLUP);
  
  pinMode(current_pinX, INPUT); //current sensor set to analogue input
  pinMode(current_pinY, INPUT);
  pinMode(current_pinZ, INPUT);
  pinMode(current_pinGrab, INPUT);
  pinMode(current_pinRot, INPUT);

  Rotation.attach(inRot); //enable servo pin 
  Rotation.write(0); //initially set to 0deg 

  Serial.begin(57600); //enable serial communication for debugging
  
  myPIDX.SetMode(AUTOMATIC); //turn PID on
  myPIDY.SetMode(AUTOMATIC);
  myPIDZ.SetMode(AUTOMATIC);
  myPIDGrab.SetMode(AUTOMATIC);
  
  myPIDX.SetTunings(KpX, KiX, KdX); //adjust PID gains
  myPIDY.SetTunings(KpY, KiY, KdY);
  myPIDZ.SetTunings(KpZ, KiZ, KdZ);
  myPIDGrab.SetTunings(KpGrab, KiGrab, KdGrab);

  switch_statusX = digitalRead(zero_switchX);
  switch_statusY = digitalRead(zero_switchY);
  switch_statusZ = digitalRead(zero_switchZ);
  switch_statusGrab = digitalRead(zero_switchGrab);
  
//  while(switch_statusX == 0){ //send motor to initial position touching switch
//    analogWrite(in2X, 255); //backwards
//    digitalWrite(in1X, LOW); //new value of speed
//    }
//  myEncX.write(0); //zero encoder position
//
//  while(switch_statusY == 0){ //send motor to initial position touching switch
//    analogWrite(in2Y, 255); //backwards
//    digitalWrite(in1Y, LOW); //new value of speed
//    }
//  myEncY.write(0); //zero encoder position
//
//  while(switch_statusZ == 0){ //send motor to initial position touching switch
//    analogWrite(in2Z, 255); //backwards
//    digitalWrite(in1Z, LOW); //new value of speed
//    }
//  myEncZ.write(0); //zero encoder position
//
//  while(switch_statusGrab == 0){ //send motor to initial position touching switch
//    analogWrite(in2Grab, 255); //backwards
//    digitalWrite(in1Grab, LOW); //new value of speed
//    }
//  myEncGrab.write(0); //zero encoder position
}




void loop() {

  // update the motor positions

  double x_pos = update_x_pos();

  double y_pos = update_y_pos();

  double z_pos = update_z_pos();

  double yaw_pos = update_yaw_pos();

  double grab_pos = update_grab_pos();



  // measure the currents

  double x_trq = measure_x_trq();

  double y_trq = measure_y_trq();

  double z_trq = measure_z_trq();

  double yaw_trq = measure_yaw_trq();

  double grab_trq = measure_grab_trq();



  // Update ROS

  update_ros(x_pos, y_pos, z_pos, yaw_pos, grab_pos, 

             x_trq, y_trq, z_trq, yaw_trq, grab_trq);

}



void setup_ros() {

#ifdef USE_ROS

  // This is all that goes here for now

  nh.initNode();

  nh.advertise(pub);

  nh.subscribe(sub);

  

#ifdef AVERAGE_READINGS

  measured_data_msg.x_pos = 0;

  measured_data_msg.y_pos = 0;

  measured_data_msg.z_pos = 0;

  measured_data_msg.theta_pos = 0;

  measured_data_msg.grabber_pos = 0;

  

  measured_data_msg.x_torque = 0;

  measured_data_msg.y_torque = 0;

  measured_data_msg.z_torque = 0;

  measured_data_msg.theta_torque = 0;

  measured_data_msg.grabber_torque = 0;



  average_counter = 0;

#endif // AVERAGE_READINGS



  ros_timer = millis();

#endif // USE_ROS

}



void update_ros(double x_pos, double y_pos, double z_pos, double yaw_pos, double grab_pos,

                double x_trq, double y_trq, double z_trq, double yaw_trq, double grab_trq) {

#ifdef USE_ROS

#ifdef AVERAGE_READINGS

  measured_data_msg.x_pos += x_pos;

  measured_data_msg.y_pos += y_pos;

  measured_data_msg.z_pos += z_pos;

  measured_data_msg.theta_pos += yaw_pos;

  measured_data_msg.grabber_pos += grab_pos;

  

  measured_data_msg.x_torque += x_pos;

  measured_data_msg.y_torque += y_pos;

  measured_data_msg.z_torque += z_pos;

  measured_data_msg.theta_torque += yaw_pos;

  measured_data_msg.grabber_torque += grab_pos;



  average_counter++;

#else

  measured_data_msg.x_pos = x_pos;

  measured_data_msg.y_pos = y_pos;

  measured_data_msg.z_pos = z_pos;

  measured_data_msg.theta_pos = yaw_pos;

  measured_data_msg.grabber_pos = grab_pos;

  

  measured_data_msg.x_torque = x_trq;

  measured_data_msg.y_torque = y_trq;

  measured_data_msg.z_torque = z_trq;

  measured_data_msg.theta_torque = yaw_trq;

  measured_data_msg.grabber_torque = grab_trq;

#endif // AVERAGE_READINGS



  if(millis() - ros_timer > 1.0f / PUBLISH_FREQ) {

    ros_timer = millis();

#ifdef AVERAGE_READINGS

    measured_data_msg.x_pos /= (double)average_counter;

    measured_data_msg.y_pos /= (double)average_counter;

    measured_data_msg.z_pos /= (double)average_counter;

    measured_data_msg.theta_pos /= (double)average_counter;

    measured_data_msg.grabber_pos /= (double)average_counter;

    

    measured_data_msg.x_torque /= (double)average_counter;

    measured_data_msg.y_torque /= (double)average_counter;

    measured_data_msg.z_torque /= (double)average_counter;

    measured_data_msg.theta_torque /= (double)average_counter;

    measured_data_msg.grabber_torque /= (double)average_counter;

#endif // AVERAGE_READINGS

    pub.publish(&measured_data_msg);



#ifdef AVERAGE_READINGS

    measured_data_msg.x_pos = 0;

    measured_data_msg.y_pos = 0;

    measured_data_msg.z_pos = 0;

    measured_data_msg.theta_pos = 0;

    measured_data_msg.grabber_pos = 0;

    

    measured_data_msg.x_torque = 0;

    measured_data_msg.y_torque = 0;

    measured_data_msg.z_torque = 0;

    measured_data_msg.theta_torque = 0;

    measured_data_msg.grabber_torque = 0;

    average_counter = 0;

#endif // AVERAGE_READINGS

  }



  // Check for new messages

  nh.spinOnce();

#endif // USE_ROS

}



#ifdef USE_ROS

void frame_target_callback(const cynaptix::FrameTarget& msg) {

  targetX = msg.x_pos;

  targetY = msg.y_pos;

  targetZ = msg.z_pos;

  targetRot = msg.theta_pos;

  targetGrab = msg.grabber_pos;

  char buf[64];
  memset(buf, '\0', 64);
  sprintf(buf, "Received msg: [%d, %d, %d, %d, %d]", (int)targetX, (int)targetY, (int)targetZ, (int)targetRot, (int)targetGrab);
  nh.loginfo(buf);
}

#endif // USE_ROS



double update_x_pos() {

    encod_countX = myEncX.read(); //get instantaneous position
    if (encod_countX != targetX){
      myPIDX.Compute(); //calculate PID output

        if (encod_countX < targetX){
          analogWrite(in2X, speed_X); //backwards
          digitalWrite(in1X, LOW); //new value of speed

        } else {
          analogWrite(in2X, speed_X); //forwards
          digitalWrite(in1X, HIGH); //new value of speed
        }
    } else {
     digitalWrite(in1X, LOW);
     digitalWrite(in2X, LOW);
    }

  return encod_countX;

}

double update_y_pos() {

    encod_countY = myEncY.read(); //get instantaneous position
    if (encod_countY != targetY){
      myPIDY.Compute(); //calculate PID output

        if (encod_countY < targetY){
          analogWrite(in2Y, speed_Y); //backwards
          digitalWrite(in1Y, LOW); //new value of speed

        } else {
          analogWrite(in2Y, speed_Y); //forwards
          digitalWrite(in1Y, HIGH); //new value of speed
        }
    } else {
     digitalWrite(in1Y, LOW);
     digitalWrite(in2Y, LOW);
    }

  return encod_countY;

}

double update_z_pos() {

    encod_countZ = myEncZ.read(); //get instantaneous position
    if (encod_countZ != targetZ){
      myPIDZ.Compute(); //calculate PID output

        if (encod_countZ < targetZ){
          analogWrite(in2Z, speed_Z); //backwards
          digitalWrite(in1Z, LOW); //new value of speed

        } else {
          analogWrite(in2Z, speed_Z); //forwards
          digitalWrite(in1Z, HIGH); //new value of speed
        }
    } else {
     digitalWrite(in1Z, LOW);
     digitalWrite(in2Z, LOW);
    }

  return encod_countZ;

}

double update_yaw_pos() {

Rotation.write(targetRot); //send servo to target angle between 0-180deg

  return 0;

}

double update_grab_pos() {

        encod_countGrab = myEncGrab.read(); //get instantaneous position
    if (encod_countGrab != targetGrab){
      myPIDGrab.Compute(); //calculate PID output

        if (encod_countGrab < targetGrab){
          analogWrite(in2Grab, speed_Grab); //backwards
          digitalWrite(in1Grab, LOW); //new value of speed

        } else {
          analogWrite(in2Grab, speed_Grab); //forwards
          digitalWrite(in1Grab, HIGH); //new value of speed
        }
    } else {
     digitalWrite(in1Grab, LOW);
     digitalWrite(in2Grab, LOW);
    }

  return encod_countGrab;

}



double measure_x_trq() {

    VoutX = analogRead(current_pinX); //read voltage output from current sensor
    currentX = VoutX * (5.0/1023.0) * 0.4; //convert Vout into current measured
    torqueX = currentX * 1;//?; //multiply current by motor constant to get torque 

  return torqueX;

}

double measure_y_trq() {

    VoutY = analogRead(current_pinY); //read voltage output from current sensor
    currentY = VoutY * (5.0/1023.0) * 0.4; //convert Vout into current measured
    torqueY = currentY * 1;//?; //multiply current by motor constant to get torque

  return torqueY;

}

double measure_z_trq() {

    VoutZ = analogRead(current_pinZ); //read voltage output from current sensor
    currentZ = VoutZ * (5.0/1023.0) * 0.4; //convert Vout into current measured
    torqueZ = currentZ * 1;//?; //multiply current by motor constant to get torque

  return torqueZ;

}

double measure_yaw_trq() {

    VoutRot = analogRead(current_pinRot); //read voltage output from current sensor
    currentRot = VoutRot * (5.0/1023.0) * 0.4; //convert Vout into current measured
    torqueRot = currentRot * 1;//?; //multiply current by motor constant to get torque

  return torqueRot;

}

double measure_grab_trq() {

    VoutGrab = analogRead(current_pinGrab); //read voltage output from current sensor
    currentGrab = VoutGrab * (5.0/1023.0) * 0.4; //convert Vout into current measured
    torqueGrab = currentGrab * 1;//?; //multiply current by motor constant to get torque

  return torqueGrab;

}
