#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

import math
import numpy as np
import cv2
import cv_bridge

bridge = cv_bridge.CvBridge()

MIN_AREA_TRACK = 300
MAX_AREA_TRACK = 10000
LINEAR_SPEED = 0.2
KP = 1.5/100
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.06
SEARCH_TIMEOUT = int(1.0 / TIMER_PERIOD)
SEARCH_ANGULAR_SPEED = 0.5
LOST_LINEAR_SPEED = LINEAR_SPEED * 0.4

# --- Lap completion via odometry ---
# The track is a single continuous closed curve (it crosses itself
# visually a few times, but there's only one way to trace it). So "lap
# complete" is simply: got far enough from start, then came back close
# to start.
MIN_DISTANCE_AWAY = 1.0
RETURN_RADIUS = 0.3
RETURNS_REQUIRED = 2   # ignore the first return near spawn (happens before the loop is actually entered) - only stop on the 2nd

lower_bgr_values = np.array([100, 100, 100])
upper_bgr_values = np.array([200, 200, 200])

def crop_size(height, width):
    return (height//10, 5*height//6, width//8, 7*width//8)

image_input = 0
error = 0
just_seen_line = False
should_move = False
lost_frame_count = 0
last_line_pos = None    # (x, y) of the line's last known position
last_line_vel = (0, 0)  # estimated (dx, dy) between the last two frames

start_x = None
start_y = None
current_x = None
current_y = None
has_traveled_away = False
lap_completing = False
return_count = 0


def start_follower_callback(request, response):
    global should_move, lost_frame_count, last_line_pos, last_line_vel
    global start_x, start_y, has_traveled_away, lap_completing, return_count
    should_move = True
    lost_frame_count = 0
    last_line_pos = None
    last_line_vel = (0, 0)
    start_x = current_x
    start_y = current_y
    has_traveled_away = False
    lap_completing = False
    return_count = 0
    return response

def stop_follower_callback(request, response):
    global should_move
    should_move = False
    return response

def image_callback(msg):
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def odom_callback(msg):
    global current_x, current_y
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

def get_contour_data(mask, out):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    line = {}
    line_candidates = []

    for contour in contours:
        M = cv2.moments(contour)
        area = M['m00']
        if area < MIN_AREA_TRACK or area > MAX_AREA_TRACK:
            continue
        cx = int(M['m10'] / area)
        cy = int(M['m01'] / area)
        cx_full = cx + crop_w_start
        line_candidates.append((area, cx_full, cy))
        cv2.circle(out, (cx, cy), 5, (0, 255, 0), -1)

    if not line_candidates:
        return line

    global last_line_pos, last_line_vel

    if last_line_pos is not None:
        # At a crossing, more than one contour can appear at once (the
        # petal loop and the main arc both passing through). Match
        # against where the line WAS last frame -- not an extrapolated
        # future position based on velocity. Velocity-based prediction
        # is biased toward whichever branch continues smoothly (low
        # curvature), which on this track means it keeps skipping the
        # sharp turn into each petal and just follows the gentle outer
        # arc. Matching to the last actual position instead has no such
        # bias -- it just follows wherever the physical line currently
        # is, including a tight turn into a petal when that's genuinely
        # where the line goes.
        _, cx_full, cy = min(
            line_candidates,
            key=lambda c: math.hypot(c[1] - last_line_pos[0], c[2] - last_line_pos[1])
        )
    else:
        _, cx_full, cy = max(line_candidates, key=lambda c: c[0])

    last_line_vel = (cx_full - last_line_pos[0], cy - last_line_pos[1]) if last_line_pos is not None else (0, 0)
    last_line_pos = (cx_full, cy)
    line['x'] = cx_full
    line['y'] = cy

    return line

def timer_callback():
    global error, image_input, just_seen_line, should_move, lost_frame_count
    global has_traveled_away, lap_completing, return_count

    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape
    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    output = image
    line = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])

    message = Twist()

    if line:
        error = line['x'] - width / 2
        just_seen_line = True
        lost_frame_count = 0
        message.linear.x = LINEAR_SPEED
    else:
        if just_seen_line:
            error = error * LOSS_FACTOR
            just_seen_line = False
        lost_frame_count += 1
        message.linear.x = LOST_LINEAR_SPEED

    # --- Lap completion (odometry-based) ---
    if should_move and start_x is not None and current_x is not None and not lap_completing:
        dist_from_start = math.hypot(current_x - start_x, current_y - start_y)
        if not has_traveled_away:
            if dist_from_start > MIN_DISTANCE_AWAY:
                has_traveled_away = True
                print(f"[DEBUG] traveled away from start (dist={dist_from_start:.2f}m)", flush=True)
        else:
            if dist_from_start < RETURN_RADIUS:
                return_count += 1
                if return_count >= RETURNS_REQUIRED:
                    lap_completing = True
                    print(f"[DEBUG] LAP COMPLETE (dist={dist_from_start:.2f}m, return #{return_count})", flush=True)
                else:
                    # This return is happening before the loop was
                    # actually entered -- don't stop, just note it and
                    # re-arm so the next real return counts separately.
                    has_traveled_away = False
                    print(f"[DEBUG] return #{return_count} near spawn (dist={dist_from_start:.2f}m) - "
                          f"not stopping yet, {RETURNS_REQUIRED - return_count} more needed", flush=True)

    if should_move:
        if lost_frame_count > SEARCH_TIMEOUT:
            message.linear.x = 0.0
            message.angular.z = -SEARCH_ANGULAR_SPEED if error > 0 else SEARCH_ANGULAR_SPEED
        else:
            message.angular.z = -error * KP
    else:
        message.linear.x = 0.0
        message.angular.z = 0.0

    if lap_completing:
        message.linear.x = 0.0
        message.angular.z = 0.0

    publisher.publish(message)
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
    cv2.imshow("output", output)
    cv2.waitKey(5)

    if lap_completing:
        should_move_stop()

def should_move_stop():
    global should_move
    if should_move:
        should_move = False
        print("Track Completed")
        empty_message = Twist()
        publisher.publish(empty_message)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    global node
    node = Node('follower')

    global publisher
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
    node.create_subscription(Image, 'camera/image_raw', image_callback, rclpy.qos.qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom_callback, rclpy.qos.qos_profile_sensor_data)
    node.create_timer(TIMER_PERIOD, timer_callback)
    node.create_service(Empty, 'start_follower', start_follower_callback)
    node.create_service(Empty, 'stop_follower', stop_follower_callback)

    rclpy.spin(node)

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    publisher.publish(empty_message)
    node.destroy_node()
    rclpy.shutdown()
    exit()
