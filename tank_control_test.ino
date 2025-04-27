#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Ultrasonic.h>  // Include the ultrasonic sensor library

// Motor Pins (ESP32 GPIO)
#define LEFT_MOTOR_PWM 4
#define RIGHT_MOTOR_PWM 16
#define LEFT_MOTOR_IN1 12
#define LEFT_MOTOR_IN2 13
#define RIGHT_MOTOR_IN3 14
#define RIGHT_MOTOR_IN4 15

// Ultrasonic Sensor Pins
#define TRIG_PIN 17
#define ECHO_PIN 18
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

// ROS Node Handle
ros::NodeHandle nh;

// Define the distance threshold for obstacle detection
#define OBSTACLE_THRESHOLD 20  // in cm
#define BACKUP_DURATION 1000   // milliseconds
#define TURN_DURATION 500      // milliseconds

// Function to map velocity to PWM
int velocityToPWM(float vel) {
    return constrain(abs(vel) * 255, 0, 255);
}

// Function to handle incoming Twist messages
void cmdVelCallback(const geometry_msgs::Twist &msg) {
    float linear_x = msg.linear.x;  // Forward (+) / Backward (-)
    float angular_z = msg.angular.z; // Left (+) / Right (-)

    // Get the distance from the ultrasonic sensor
    long distance = ultrasonic.read();  // Distance in cm
    
    // If an obstacle is too close (e.g., less than 20 cm), take evasive action
    if (distance < OBSTACLE_THRESHOLD) {
        // Back up for a short duration
        moveBackwards();
        delay(BACKUP_DURATION);

        // Turn for a short duration
        turnAround();
        delay(TURN_DURATION);

        // After evasive actions, reset to 0 speed before resuming cmd_vel
        linear_x = 0;
        angular_z = 0;
    }

    // Compute motor speeds
    float left_speed = linear_x - angular_z;
    float right_speed = linear_x + angular_z;

    int left_pwm = velocityToPWM(left_speed);
    int right_pwm = velocityToPWM(right_speed);

    // Set motor directions
    if (left_speed > 0) {
        digitalWrite(LEFT_MOTOR_IN1, HIGH);
        digitalWrite(LEFT_MOTOR_IN2, LOW);
    } else {
        digitalWrite(LEFT_MOTOR_IN1, LOW);
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
    analogWrite(RIGHT_MOTOR_PWM, right_pwm);
}

// Function to move the robot backward (for obstacle avoidance)
void moveBackwards() {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, HIGH);

    // Set speed to a moderate value for backup
    analogWrite(LEFT_MOTOR_PWM, 200);
    analogWrite(RIGHT_MOTOR_PWM, 200);
}

// Function to turn the robot in place (for obstacle avoidance)
void turnAround() {
    // Turn right for 1 second (you can adjust this for left turn if necessary)
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, HIGH);

    // Set speed to a moderate value for turn
    analogWrite(LEFT_MOTOR_PWM, 150);
    analogWrite(RIGHT_MOTOR_PWM, 150);
}

// ROS Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
    nh.initNode();
    nh.subscribe(sub);

    // Motor pin modes
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

void loop() {
    nh.spinOnce();  // Process ROS messages
    delay(10);
}
