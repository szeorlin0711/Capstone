#include <ros.h>
#include <geometry_msgs/Twist.h>

// Motor Pins
#define LEFT_MOTOR_PWM 4
#define RIGHT_MOTOR_PWM 16
#define LEFT_MOTOR_IN1 12
#define lEFT_MOTOR_IN2 13
#define RIGHT_MOTOR_IN3 14
#define RIGHT_MOTOR_IN4 15

// ROS Node Handle
ros::NodeHandle nh;

// Fucntion to map velocity to PWM
int velocityToPWM(float vel) {
  return constrain(abs(vel) * 255, 0, 255);
}

// Function to handle incoming twist messages
void cmdVelCallback(const geometry_msgs::Twist &msg) {
  float linear_x = msg.linear.x;    // Forward and backawrd
  float angluat_z = msg.angulat.z;  // Left and right

  // Compute Motor Speeds
  float left_speed = linear_x - angular_z;
  float right_speed = linear_x + angular_z;

  int left_pwm = velocityToPWM(left_speed);
  int right_pwm = velocityToPWM(right_speed);

  // Set motor directions
  if (left_speed > 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  }else {
    digitaWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
  }

  if (right_speed > 0) {
    digitalWrite(RIGHT_MOTOR_IN3, HIGH);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  }

  // Set motor speeds (PWM)
  analogWrite(LEFT_MOTOR_PWM, left_pwm);
  analotWrite(RIGHT_MOTOR_PWM, right_pwm);
}

// ROS Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  // Motor pin nodes
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);

  // Stop motors initially
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}

void loop(){
  nh.spinOnce(); // Process ROS messages
  delay(10);
}