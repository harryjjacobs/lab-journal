#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h>

using namespace ros;

NodeHandle nh;
Servo servo1;
Servo servo2;
Servo servo3;

void cb(const sensor_msgs::JointState& msg) {
  servo1.write(msg.position[0]*180/PI); // 0-180 (convert from rad to deg)
  servo2.write(msg.position[1]*180/PI); // 0-180
  servo3.write(msg.position[2]*180/PI); // 0-180
}

Subscriber<sensor_msgs::JointState> sub("joint_states", cb);

void setup() {
  nh.getHardware()->setBaud(115200); // needs to be high otherwise it doesn't work
  nh.initNode();
  nh.subscribe(sub);

  servo1.attach(6);  // attaches the servo on pin 6 to the servo object
  servo2.attach(5);
  servo3.attach(10);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
