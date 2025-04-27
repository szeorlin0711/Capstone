#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rft_hsvrgb as rf
import imutils as imut
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

CALIBRATION_FILE = "calibration_data.npy"

yellow_detected = False

def color_callback(msg):
    global yellow_detected
    color = msg.data.lower()

    if color == "yellow":
        yellow_detected = True
    else:
        yellow_detected = False

def save_calibration_data(color_ranges, color_space='HSV'):
    """save the calibrated color ranges to a file"""
    data = {'color_space': color_space, 'ranges': color_ranges}
    np.save(CALIBRATION_FILE, data)
    print(f"Calibration data saved in {color_space} space.")

def load_calibration_data():
    """load color ranges from the saved calibration file."""
    try:
        data = np.load(CALIBRATION_FILE, allow_pickle=True).item()
        return data.get('ranges', {}), data.get('color_space', 'HSV')
    except FileNotFoundError:
        return None, None

def process_frame(frame, lower, upper, range_filter='HSV'):
    """Process frame to find and follow the line"""
    if range_filter.upper() == 'HSV':
        frame_to_thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    elif range_filter.upper() == 'BGR':
        frame_to_thresh = frame
    else:
        print(f"Error: Unsupported color space {range_filter}")
        return frame, 0, 0

    mask = cv2.inRange(frame_to_thresh, lower, upper)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    edges = cv2.Canny(mask, 50, 150, apertureSize=3)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                            minLineLength=50, maxLineGap=10)

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    v, w = 0, 0

    if lines is not None:
        longest_line = None
        max_length = 0
        for line in lines:
            for x1, y1, x2, y2 in line:
                length = math.hypot(x2 - x1, y2 - y1)
                if length > max_length:
                    max_length = length
                    longest_line = (x1, y1, x2, y2)

        if longest_line is not None:
            x1, y1, x2, y2 = longest_line

            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

            if y1 < y2:
                target_point = (x1, y1)
            else:
                target_point = (x2, y2)

            tx, ty = target_point

            cv2.circle(frame, target_point, 5, (0, 0, 255), -1)
            cv2.arrowedLine(frame, (center_x, center_y), target_point, (255, 0, 0), 2)

            error_x = tx - center_x
            error_y = center_y - ty

            k_linear = 0.005
            v = k_linear * error_y

            k_turn = 0.005
            w = k_turn * error_x

            print(f"Target point: {target_point}, error_x: {error_x}, error_y: {error_y}")
            print(f"Command velocities: linear = {v:.2f}, angular = {w:.2f}")

    else:
        print("No line detected.")

    return frame, v, w

# --- GLOBAL VARIABLES ---
current_detected_color = None
color_ranges = {}
range_filter = 'HSV'

def color_callback(msg):
    """Callback to update the color to track based on /detected_color topic"""
    global current_detected_color
    detected_color = msg.data.lower()
    if detected_color in color_ranges:
        current_detected_color = detected_color
        rospy.loginfo(f"Updated tracking color to {current_detected_color}")
    else:
        rospy.logwarn(f"Detected color {detected_color} not found in calibrated ranges.")

def main():
    global color_ranges, range_filter

    global yellow_detected  # Added global for yellow detection

    rospy.init_node("line_follower")
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/detected_color", String, color_callback)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Load calibration
    color_ranges, saved_color_space = load_calibration_data()

    if color_ranges:
        user_input = input("Calibration data found. Use saved data? (y/n): ").strip().lower()
        if user_input == 'y':
            range_filter = saved_color_space
            print(f"Using saved calibration in {range_filter} space.")
        else:
            range_filter = input("Select color space (HSV/BGR): ").strip().upper()
    else:
        range_filter = input("No saved calibration. Select color space (HSV/BGR): ").strip().upper()

    if range_filter not in ['HSV', 'BGR']:
        print("Invalid color space selected. Exiting.")
        return

    if not color_ranges:
        print("No calibration data available. Exiting.")
        return

    print("Waiting for detected color to be published...")
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # If yellow is detected, stop tracking
        if yellow_detected:
            print("Yellow detected! Pausing line tracking.")
            twist_msg = Twist()
            twist_msg.linear.x = 0  # Stop forward movement
            twist_msg.angular.z = 0  # Stop turning
            cmd_vel_pub.publish(twist_msg)  # Publish stop command
        else:
            if current_detected_color is not None:
                lower, upper = color_ranges[current_detected_color]
                processed_frame, v, w = process_frame(frame, lower, upper, range_filter)

                twist_msg = Twist()
                twist_msg.linear.x = v
                twist_msg.angular.z = w

                cmd_vel_pub.publish(twist_msg)
                print(f"Published Twist: linear={v:.2f}, angular={w:.2f}")

                resized_frame = imut.resize(processed_frame, width=720)
                cv2.imshow("Line Tracking", resized_frame)
            else:
                # Show raw frame until color is detected
                resized_frame = imut.resize(frame, width=720)
                cv2.imshow("Line Tracking", resized_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



