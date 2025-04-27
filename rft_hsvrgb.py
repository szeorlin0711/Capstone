import cv2
import imutils
import os

switch = True
CONFIG_FILE = "config.txt"

def save_color_space(color_space):
    try:
        with open(CONFIG_FILE, "w") as f:
            f.write(color_space)
    except Exception as e:
        print(f"Error saving config: {e}")

def load_color_space():
    if os.path.exists(CONFIG_FILE):
        try:    
            with open(CONFIG_FILE, "r") as f:
                return f.read().strip()
        except Exception as e:
            print(f"Error loading config: {e}")    
            return "BGR" #Default if no config file exists
    return "BGR"

def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
        for j in range_filter:
            cv2.createTrackbar(f"{j}_{i}", "Trackbars", v, 255, callback)


def get_trackbar_values(range_filter):

    values = []
    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos(f"{j}_{i}", "Trackbars")
            values.append(v)
    return values


def process_live_feed(camera, preview=True, imut=False, frame_width=1080):
    global switch
    range_filter = load_color_space()
    switch = range_filter == "BGR"
    setup_trackbars(range_filter.upper())

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame.")
            break
            
        # Resize the frame if necessary
        if imut:
            frame = imutils.resize(frame, width=frame_width)

        # Convert to HSV or keep as BGR based on the selected color space
        frame_to_thresh = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) if range_filter == 'HSV' else frame
        
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter.upper())
        
        # Thresholding based on trackbar values
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
        
        if preview:
            preview_image = cv2.bitwise_and(frame, frame, mask=thresh)
            preview_image = imutils.resize(frame, width=frame_width)
            cv2.imshow("Preview", preview_image)

        # Show the thresholded result
        thresh_resized = cv2.bitwise_and(frame, frame, mask=thresh)
        thresh_resized = imutils.resize(thresh_resized, width=frame_width)
        cv2.imshow("Thresh", thresh_resized)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            switch = not switch
            cv2.destroyWindow("Trackbars")
            if switch:
                range_filter = 'BGR'
            else:
                range_filter = 'HSV'
            setup_trackbars(range_filter.upper())
        
        if key == 32:  # Spacebar to save and exit
            save_color_space(range_filter)  # Save the current space before exiting
            break

    cv2.destroyAllWindows()

    return [v1_min, v2_min, v3_min, v1_max, v2_max, v3_max]#, range_filter #remove the range_filter part when running colorshapetracker


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    minmax = process_live_feed(cap, preview=False, imut=True, frame_width=720)
    cap.release()