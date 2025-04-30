#include <ros.h>
#include <std_msgs/Bool.h>

// Ultrasonic Sensor Pins
#define TRIG_PIN 13  // TRIG
#define ECHO_PIN 12  // ECHO

// Distance threshold (in cm)
#define OBSTACLE_THRESHOLD 16

// ROS Node Handle
ros::NodeHandle nh;

// Message and Publisher
std_msgs::Bool obstacle_msg;
ros::Publisher obstacle_pub("in_range", &obstacle_msg);

// Function to read ultrasonic distance
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

    return duration / 58.0; // Convert to cm
}

void setup() {
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(obstacle_pub);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    //Serial.begin(115200);
    while (!Serial);

    Serial.println("Ultrasonic Sensor Node Initialized.");
}

void loop() {
    long distance = readUltrasonicDistance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    obstacle_msg.data = (distance > 0 && distance < OBSTACLE_THRESHOLD);
    obstacle_pub.publish(&obstacle_msg);

    nh.spinOnce();
    delay(100);  // Delay to control publish rate
}
