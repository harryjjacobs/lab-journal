# Lab journal for ROCO222 module

## Markdown

Markdown is a formatting language for plain text where the syntax is also plain text. It is designed to be easily converted to HTML.

Github supports additional markdown syntax and details are listed here: [markdown cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)

A # denotes a header, multiple hashes (##) make smaller and smaller headers (H1, H2 etc).

Two underscores or asterisks on either side of the word will make it __bold__ and An underscore or asterisk on either side will make _italic_. (Really these are just defined as 'emphasis' in the markdown spec rather than bold or italic).

The syntax can be combined to make bold and italic text.

* A single asterisk at the start of the line followed by text makes an unordered list item

+ A plus...

- or a minus also denote an unordered list

1. An ordered list is denoted by the number 1 followed by a fullstop. (1.)

A horizontal rule can be drawn by either 3 dashes, 3 asterisks or 3 underscores:

---

***

___


Line breaks are just created by newline characters (hitting enter once will insert 1 newline)

Links are denoted by the display text in square brackets followed by the url in normal brackets without a space. [This is a link](www.google.com)

The syntax for images is similar to links but with an exclamation mark at the start. The text in the square brackets is the alt text.
![alt text](www.example.com/image.jpg)

You can also define the image as a reference where you define a variable for the image url (in square brackets followed by a colon and then the url. ie: [logo]: image.png) and use this in the brackets.
![alt text][logo]
[logo]: www.example.com/image.jpg

> Block quotes are denoted by a > symbol.
> This is an example

`inline code is represented by back-ticks around the text`

HTML can also be used within markdown.

## Command line
Some common commands:
`cd, ls, mkdir` etc
`ls -al` will show hidden files (files with a . at the front)

## Git
Git is a version control system.
`git init` creates a git repository in the current directory.

`git add` marks files to be committed.

`git commit` commits file changes to the local repo.

`git commit -m <message>` allows you to specify a message for the commit.

`git add <name> <url of remote>` is used to add a remote (usually named origin).

`git push <remote name>` is used to push changes to a remote git server.

`git push -u origin master/images` push the local master/images branch to the remote called origin with the 'upstream' flag set.



## Hacking into the robot task
Googled details about the Aldebaran nao and discovered the default hostname of nao.local and the default username and password which are 'nao'.

The robot it called chapman so worked out the hostname is chapman.local.

ssh'ed in with 'ssh nao@chapman.local' and typed in the password.

Ran the commands using the python command line interface:

```
from naoqi import ALProxy
tts = ALProxy("ALTextToSpeech", "localhost", 9559)
tts.say("I've hacked you, robot!")
```


---


## Servo Project

![finished arm](https://github.com/harryjjacobs/lab-journal/blob/master/images/finished-arm.jpg "Picture of the completed arm")

You can find the Solidworks files I used to make my model in the 'solidworks' directory and the STL files from it in the 'stl' directory. The Arduino code can be found inside the 'robot-arm-arduino' directory.

### Servo motors
The following code does a low frequency (0.2Hz) sine wave between 0..180 degrees on the servo:
```
#include <Servo.h>

Servo servo;

int i = 0;
int del = 1000/0.2/360; // 0.2Hz over 360 degrees(2*180 spins)
void setup() {
  servo.attach(10);
}

void loop() {
  for (i = 0; i <= 360; i++) { 
    servo.write((sin(i*PI/180)+1)*180/2); // convert i to radians, get the sin of it, scale it up to between 0..180
    delay(del);
  }
}
```
![low frequency servo sweep](https://github.com/harryjjacobs/lab-journal/blob/master/images/servo-sweep.gif "Gif of low frequency servo sweep")

### Using ROS.
Before doing anything, you must run `roscore`

These commands will block the terminal so you should open a new one or append '&' after the command.

Once it is running you can publish messages on a 'topic':
`rostopic pub /test std_msgs/String "hello"`

In this case we are publishing a message of type String on the topic 'test'.

You can listen and print messages sent on this topic with:
`rostopic echo /test`


Rviz is used for 3D visualisation in ROS.

rosserial must be installed for use with arduino (`sudo apt install ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino`)

The following code uses the Servo library and the ros library for arduino to control the
servo motor via ROS.
A node is set up to subscribe to the 'servo' topic so that whenever we receive a frame
on this topic the callback function is called containing the 16 bit unsigned integer that
is expected in the message. The 16 bit unsigned integer should be a value between 0-180
as it represents the position to move the servo to in degrees.

```
#include <ros.h>
#include <std_msgs/UInt16.h>
#include<Servo.h>

using namespace ros;

NodeHandle nh;
Servo servo;

void cb(const std_msgs::UInt16& msg) {
  servo.write(msg.data); // 0-180
}

Subscriber<std_msgs::UInt16> sub("servo", cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  nh.spinOnce();
  delay(1);
}
```

This command is used to configure ROS to output to the serial port.
`rosrun rosserial_python serial_node.py /dev/ttyACM0`

You can specify the baudrate with `_baud`, for example: `rosrun rosserial_python serial_node.py /dev/ttyS0 _baud:=115200`

---

The next part is a 3D model of the arm in RViz.

The .udrf file is an XML file containing the geometric description of the 3D object:
```
<?xml version="1.0" encoding="UTF-8"?>
<robot name="roco_arm">
   <link name="base_link">
      <visual>
         <geometry>
            <cylinder length="0.06" radius="0.1" />
         </geometry>
      </visual>
   </link>
   <link name="first_segment">
      <visual>
         <geometry>
            <box size="0.6 0.05 0.1" />
         </geometry>
         <origin rpy="0 0 0" xyz="-0.3 0 0" />
      </visual>
   </link>
   <joint name="base_to_first" type="revolute">
      <axis xyz="0 1 0" />
      <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
      <parent link="base_link" />
      <child link="first_segment" />
      <origin xyz="0 0 0.03" />
   </joint>
</robot>
```

You load this as the description of your robot using the rosparam command:
`rosparam set robot_description -t models/robot-arm.urd`

You then use the rosrun command to run the 'robot_state_publisher' and the 'joint_state_publisher'
`rosrun robot_state_publisher robot_state_publisher`
(open a new terminal)
`rosrun joint_state_publisher joint_state_publisher _use_gui:=true`

The robot_state_publisher broadcasts data about the transforms from the data file (the link elements).
The joint_state_publisher reads data about the joints from the data file and creates a GUI so we can move
the joints easily and adjust the pose of the robot.

Now run `rviz` and add the `RobotModel` plugin.
Then set the Fixed Frame (top-left under Global Options) to base_link so it knows what to use as
the fixed frame.

The joint_state_publisher GUI can be used to adjust the joints.

NOTE: if you are editing the URDF file, you need to rerun the robot_state_publisher and joint_state_publisher
commands but you will need to disable and re-enable the RobotModel in rviz for it to update.

![rviz screenshot](https://github.com/harryjjacobs/lab-journal/blob/master/images/RViz%20Screenshot.png "Screenshot of RVIZ working")

To start with I used the following URDF file for my robot arm with 3 degrees of freedom:

```
<?xml version="1.0" encoding="UTF-8"?>
<robot name="roco_arm">
   <link name="base_link">
      <visual>
         <geometry>
            <cylinder length="0.06" radius="0.1" />
         </geometry>
      </visual>
   </link>

   <link name="base_rotator">
      <visual>
         <geometry>
            <cylinder length="0.03" radius="0.1" />
         </geometry>
      </visual>
   </link>

   <link name="first_segment">
      <visual>
         <geometry>
            <box size="0.6 0.05 0.1" />
         </geometry>
         <origin rpy="0 0 0" xyz="-0.3 0 0" />
      </visual>
   </link>

   <link name="second_segment">
      <visual>
         <geometry>
            <box size="0.6 0.05 0.1" />
         </geometry>
         <origin rpy="0 0 0" xyz="-0.3 0 0" />
      </visual>
   </link>
   
   <joint name="base_to_rotator" type="revolute">
      <axis xyz="0 0 1" />
      <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
      <parent link="base_link" />
      <child link="base_rotator" />
      <origin xyz="0 0 0.03" />
   </joint>

	<joint name="rotator_to_first" type="revolute">
      <axis xyz="0 1 0" />
      <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
      <parent link="base_rotator" />
      <child link="first_segment" />
      <origin xyz="0 0 0.015" />
   </joint>

   <joint name="first_to_second" type="revolute">
      <axis xyz="0 -1 0" />
      <limit effort="1000" lower="-2.07" upper="2.07" velocity="0.5" />
      <parent link="first_segment" />
      <child link="second_segment" />
      <origin xyz="-0.6 0 0" />
   </joint>

</robot>
```


### Building the arm

Annoyingly got some of the dimensions slightly wrong the first time I printed my design on the 3D printer so rather than spending ages filing out the parts I just reprinted some of them. I assembled my arm and wired it to the motor shield on the Arduino. I used the two orange PWM output plugs for the first two servos and then connected the third one to pin 10.

I decided to make the arm modular so that bits could be added on to it in any order and it could be easily extended. It has 3 degrees of freedom currently.

I then used urdf files with references to the STL meshes for the arm model. Solidworks generated this file for me but with lots of additional stuff like collision data which I removed for simplicity. Also the joint rotation wasn't working properly because I hadn't set the origins in Solidworks for each STL part when I exported the urdf and meshes. Rather than going back into Solidworks I just changed the origins and rotation data in the urdf.
To generate the data in Solidworks I used a ROS add-on for Solidworks which you can find here: [http://wiki.ros.org/sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter)

Because I didn't make a ros package for simplicity's sake I have to use the absolute path to the STL files, this is inconvenient if I ever change which machine I'm working on so in future I think I will create a ros package (the Solidworks exporter actually creates one for you when you export).

```
<?xml version="1.0"?>
<robot
  name="arm-stl">
  <link
    name="base_link">
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/harryjjacobs/Arduino/servo/models/meshes/base_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>
  <link
    name="first_link">
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="file:///home/harryjjacobs/Arduino/servo/models/meshes/first_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

  <joint
    name="first_joint"
    type="revolute">
    <origin
      xyz="0 0 0"
      rpy="0 0 0" />
    <parent
      link="base_link" />
    <child
      link="first_link" />
    <axis
      xyz="0 0 1" />
   <limit effort="1000" lower="0" upper="3.141" velocity="0.5" />
  </joint>
  <link
    name="second_link">
    <visual>
      <origin
        xyz="0 -0.07 0"
        rpy="0 1.57 1.57" />
      <geometry>
        <mesh
          filename="file:///home/harryjjacobs/Arduino/servo/models/meshes/second_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>

  <joint
    name="second_joint"
    type="revolute">
    <origin
      xyz="0 0 0.07"
      rpy="0 0 0" />
    <parent
      link="first_link" />
    <child
      link="second_link" />
    <axis
      xyz="1 0 0" />
   <limit effort="1000" lower="0" upper="3.141" velocity="0.5" />
  </joint>
  <link
    name="third_link">
    <visual>
      <origin
        xyz="0.14 0 -0.07"
        rpy="1.57 1.57 -1.57" />
      <geometry>
        <mesh
          filename="file:///home/harryjjacobs/Arduino/servo/models/meshes/third_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
  </link>
  <joint
    name="third_joint"
    type="revolute">
    <origin
      xyz="0 0.07 0.07"
      rpy="0 0 0" />
    <parent
      link="second_link" />
    <child
      link="third_link" />
    <axis
      xyz="0 0 -1" />
    <limit effort="1000" lower="0" upper="3.141" velocity="0.5" />
  </joint>
</robot>
```

You can see the rviz RobotModel display here:

![rviz model with the STL version of the URDF](https://github.com/harryjjacobs/lab-journal/blob/master/images/RViz-STL-Screenshot.png "rviz model with the STL version of the URDF")

The basic Arduino code I used to read the ros messages and move the servos is:
```
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
  nh.getHardware()->setBaud(115200); // set the baud rate
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
```

In this program I set up a Subscriber object to listen for messages of type 'JointState' on the 'joint_states' topic. When a message is read the node handle calls the callback function named 'cb' with the 'JointState' argument 'msg'. We can then read the angles of each of the joints from the position array in radians and, after converting them to degrees, we can write the values straight to the servo motors.

![first gif of arm](https://github.com/harryjjacobs/lab-journal/blob/master/images/arm-attempt-1.gif "GIF of first attempt")

I fixed the problem with the first link falling off by adding a screw:
![screw to secure first link](https://github.com/harryjjacobs/lab-journal/blob/master/images/screw-fix.jpg "Picture of screw to secure first link")

I added screws to secure the 'motor-arm' in place at each joint:
![screw to secure first link](https://github.com/harryjjacobs/lab-journal/blob/master/images/screw-at-joint.jpg "Picture of screw to secure first link")


Here is a video of the finished arm:
[https://photos.app.goo.gl/wmFRnksIS5U3Bn1m2](https://photos.app.goo.gl/wmFRnksIS5U3Bn1m2)