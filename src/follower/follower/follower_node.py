#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import numpy as np
import cv2
import cv_bridge

bridge = cv_bridge.CvBridge()

MIN_AREA = 500 
MIN_AREA_TRACK = 5000
LINEAR_SPEED = 0.2
KP = 1.5/100 
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.06
FINALIZATION_PERIOD = 4
MAX_ERROR = 30

lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    return (1*height//3, height, width//4, 3*width//4)

image_input = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None


def start_follower_callback(request, response):
    global should_move, right_mark_count, finalization_countdown
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    return response

def stop_follower_callback(request, response):
    global should_move, finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def get_contour_data(mask, out):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}

    for contour in contours:
        M = cv2.moments(contour)

        # TODO 1 — Classify Contours into Track Line vs. Lap Marker
        #
        # Using image moments (M), distinguish the track line from a lap
        # marker blob purely by contour size, and locate each one's
        # centroid. Discard anything too small to be meaningful.
        #
        # Keep in mind:
        #   - The mask was taken from a horizontally-cropped region of the
        #     frame, so any x-coordinate you compute needs to be re-based
        #     onto the full image before you store it.
        #   - Multiple marker-sized blobs can appear in one frame; only one
        #     should be kept as `mark`. Think about which one is the right
        #     one to trust.
        #   - Populate `line` and/or `mark` dicts with 'x' and 'y' keys so
        #     the rest of the function can use them.
        pass

    if mark and line:
        mark_side = "right" if mark['x'] > line['x'] else "left"
    else:
        mark_side = None

    return (line, mark_side)

def timer_callback():
    global error, image_input, just_seen_line, just_seen_right_mark
    global should_move, right_mark_count, finalization_countdown

    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape
    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    output = image
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])

    message = Twist()

    # TODO 2 — Compute Tracking Error & Handle Line Loss
    #
    # `line` (from get_contour_data) tells you whether the track was seen
    # this frame. Turn that into a tracking error and a forward-speed
    # decision:
    #   - When the line is visible, derive `error` from where the line
    #     centroid sits relative to the frame center, and drive forward.
    #   - When the line disappears, the robot shouldn't just go blind —
    #     decide what `error` should become using the fact that it was
    #     just following a curve in some direction, and stop advancing
    #     until the line is reacquired.
    #   - `just_seen_line` exists so you can tell "line lost this frame"
    #     apart from "line has been lost for a while" — use it.
    pass

    # TODO 3 — Detect Lap Completion from Marker Crossings
    #
    # `mark_side` tells you which side of the line a marker was seen on
    # this frame (or None). Use repeated right-side marker sightings to
    # detect when a full lap has been completed, and kick off a
    # finalization countdown at that point.
    #
    # Things to guard against:
    #   - The same physical marker will be visible across many consecutive
    #     frames — make sure it's only counted once per crossing.
    #   - A marker glimpsed while the robot is badly off-center is
    #     unreliable and shouldn't count.
    #   - Don't restart the countdown if one is already running.
    #   - Think about how many right-side crossings actually correspond to
    #     "one full lap done" versus just "passed the start marker."
    pass

    # TODO 4 — Implement Proportional Steering & Command Gating
    #
    # Turn `error` into an actual steering command, and make sure the
    # robot only moves when it's supposed to.
    #   - The steering response should scale with how far off-center the
    #     line is, and correct *toward* the line — think carefully about
    #     sign, or the robot will steer itself further off track.
    #   - `should_move` controls whether the follower is even active;
    #     make sure the robot never keeps moving on a stale command once
    #     it's been told to stop.
    pass

    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
    cv2.imshow("output", output)
    cv2.waitKey(5)

    if finalization_countdown is not None:
        if finalization_countdown > 0:
            finalization_countdown -= 1
        elif finalization_countdown == 0:
            should_move = False
            # Stop the robot
            empty_message = Twist()
            publisher.publish(empty_message)
            print("Track Completed")
            # Shutdown the node
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()
            return


def main():
    rclpy.init()
    global node
    node = Node('follower')

    global publisher
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
    subscription = node.create_subscription(Image, 'camera/image_raw',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)

    timer = node.create_timer(TIMER_PERIOD, timer_callback)

    start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
    stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)

    rclpy.spin(node)

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    publisher.publish(empty_message)
    node.destroy_node()
    rclpy.shutdown()
    exit()
