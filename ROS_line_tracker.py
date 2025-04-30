#!/usr/bin/env python3
import cv2
import numpy as np
import math
import os
import lt_rft_hsvrgb as rf
import imutils as imut
import rospy
import atexit
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

CALIBRATION_FILE = "calibration_data.npy"
SYNC_FLAG_FILE = "/tmp/tracker_ready.flag"

yellow_detected = False
current_detected_color = None
last_tracked_color = None
color_ranges = {}
range_filter = 'HSV'
pub = None
in_range = False

def stop_robot():
    if pub:
        stop_msg = Twist()
        pub.publish(stop_msg)
        rospy.loginfo("Robot stopped on shutdown.")

def color_callback(msg):
    global yellow_detected, current_detected_color, last_tracked_color
    color = msg.data.lower()
    if color == "yellow":
        yellow_detected = True
        current_detected_color = None
    elif color in ['red', 'green', 'blue']:
        yellow_detected = False
        current_detected_color = color
        last_tracked_color = color
    else:
        rospy.loginfo(f"Received unsupported color: {color}")

def in_range_callback(msg):
    global in_range
    in_range = msg.data

def save_calibration_data(color_ranges, color_space='HSV'):
    data = {'color_space': color_space, 'ranges': color_ranges}
    np.save(CALIBRATION_FILE, data)
    print(f"Calibration data saved in {color_space} space.")

def load_calibration_data():
    try:
        data = np.load(CALIBRATION_FILE, allow_pickle=True).item()
        return data.get('ranges', {}), data.get('color_space', 'HSV')
    except FileNotFoundError:
        return None, None

def process_frame(frame, lower, upper, range_filter='HSV'):
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

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, 50, 10)

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height - 1
    v, w = 0, 0

    if lines is not None:
        longest_line = max(lines, key=lambda line: math.hypot(line[0][2] - line[0][0], line[0][3] - line[0][1]))
        x1, y1, x2, y2 = longest_line[0]

        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        target_point = (x1, y1) if y1 < y2 else (x2, y2)
        tx, ty = target_point

        cv2.circle(frame, target_point, 5, (0, 0, 255), -1)
        cv2.arrowedLine(frame, (center_x, center_y), target_point, (255, 0, 0), 2)

        error_x = tx - center_x
        error_y = center_y - ty

        v = 0.005 * error_y
        w = 0.005 * error_x

    return frame, v, w

def main():
    global pub, color_ranges, range_filter, current_detected_color, yellow_detected

    rospy.init_node("line_follower", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/detected_color", String, color_callback)
    rospy.Subscriber("/in_range", Bool, in_range_callback)
    rospy.on_shutdown(stop_robot)

    try:
        cap = cv2.VideoCapture("/dev/video2")
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        color_ranges, saved_color_space = load_calibration_data()
        if color_ranges:
            use_saved = input("Calibration data found. Use previous calibration? (y/n): ").strip().lower()
            if use_saved == 'y':
                range_filter = saved_color_space
            else:
                range_filter = input("Select color space (HSV/BGR): ").strip().upper()
        else:
            range_filter = input("No saved calibration. Select color space (HSV/BGR): ").strip().upper()
            use_saved = 'n'

        if range_filter not in ['HSV', 'BGR']:
            print("Invalid color space. Exiting.")
            return

        if not color_ranges or use_saved == 'n':
            color_ranges = {}
            for color in ['red', 'blue', 'green']:
                input(f"Press Enter to calibrate {color.upper()} color...")
                track_vals = rf.process_live_feed(cap, preview=True, imut=False, frame_width=640)
                lower = np.array(track_vals[:3])
                upper = np.array(track_vals[3:])
                color_ranges[color] = (lower, upper)
            save_calibration_data(color_ranges, range_filter)

        # CREATE SYNC FLAG
        with open(SYNC_FLAG_FILE, "w") as f:
            f.write("done")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                break

            # SAFETY PAUSE if in range
            if in_range:
                pub.publish(Twist())
                cv2.imshow("Line Tracking", imut.resize(frame, width=720))
                rate.sleep()
                continue

            # If not in range, proceed to tracking logic
            if yellow_detected:
                if last_tracked_color and last_tracked_color in color_ranges:
                    lower, upper = color_ranges[last_tracked_color]
                    processed_frame, v, w = process_frame(frame, lower, upper, range_filter)
                    twist = Twist()
                    twist.linear.x = v
                    twist.angular.z = w
                    pub.publish(twist)
                    cv2.imshow("Line Tracking", imut.resize(processed_frame, width=720))
                else:
                    pub.publish(Twist())
                    cv2.imshow("Line Tracking", imut.resize(frame, width=720))
            elif current_detected_color and current_detected_color in color_ranges:
                lower, upper = color_ranges[current_detected_color]
                processed_frame, v, w = process_frame(frame, lower, upper, range_filter)
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = w
                pub.publish(twist)
                cv2.imshow("Line Tracking", imut.resize(processed_frame, width=720))
            else:
                cv2.imshow("Line Tracking", imut.resize(frame, width=720))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()
    finally:
        stop_robot()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

@atexit.register
def cleanup_flag():
    try:
        os.remove(SYNC_FLAG_FILE)
    except FileNotFoundError:
        pass
