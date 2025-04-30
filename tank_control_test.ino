#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

//Motor Pins (Smartcar Shield)
#define LEFT_MOTOR_PWM 5   // ENA
#define RIGHT_MOTOR_PWM 11 // ENB
#define LEFT_MOTOR_IN1 7
#define LEFT_MOTOR_IN2 6
#define RIGHT_MOTOR_IN3 4
#define RIGHT_MOTOR_IN4 3

//Ultrasonic Sensor Pins (Smartcar Shield)
#define TRIG_PIN 13  // TRIG
#define ECHO_PIN 12  // ECHO

// ROS Node Handle
ros::NodeHandle nh;

// Distance threshold for detecting obstacle (in cm)
#define OBSTACLE_THRESHOLD 16

// Message to publish obstacle presence
std_msgs::Bool obstacle_msg;
ros::Publisher obstacle_pub("in_range", &obstacle_msg);

// Function to map velocity to PWM
int velocityToPWM(float vel) {
    return constrain(abs(vel) * 255, 0, 255);
}

// Manual function to read distance from ultrasonic sensor
long readUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout after 30ms
    if (duration == 0) {
        return 999; // No valid reading
    }

    long distance = duration / 58.0; // Convert duration to cm
    return distance;
}

// Function to handle incoming Twist messages
void cmdVelCallback(const geometry_msgs::Twist &msg) {
    float linear_x = msg.linear.x;
    float angular_z = msg.angular.z;

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

// ROS Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(obstacle_pub);

    // Motor pin modes
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN3, OUTPUT);
    pinMode(RIGHT_MOTOR_IN4, OUTPUT);

    // Ultrasonic sensor pin modes
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Setup Serial Monitor
    //Serial.begin(115200); //uncomment for base testing.
    while (!Serial); // Wait for Serial to be ready

    // Stop motors initially
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);

    Serial.println("Tank Control with Ultrasonic Sensor Initialized.");
}

void loop() {
    // Read ultrasonic sensor
    long distance = readUltrasonicDistance();

    // Print distance to Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Publish obstacle message
    if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
        obstacle_msg.data = true;
    } else {
        obstacle_msg.data = false;
    }
    obstacle_pub.publish(&obstacle_msg);

    nh.spinOnce();
    delay(50);  // Small delay to avoid flooding ROS
}
