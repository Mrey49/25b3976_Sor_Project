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

MIN_AREA = 150
MIN_AREA_TRACK = 300
MAX_AREA_TRACK = 10000   # a real line contour shouldn't approach filling the whole ROI
LINEAR_SPEED = 0.2
KP = 1.5/100
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.06
FINALIZATION_PERIOD = 4
MAX_ERROR = 30
SEARCH_TIMEOUT = int(1.0 / TIMER_PERIOD)   # ~1 second of holding the last turn before actively searching
SEARCH_ANGULAR_SPEED = 0.5
LOST_LINEAR_SPEED = LINEAR_SPEED * 0.4     # reduced (not zero) forward speed while the line is lost
ERROR_SMOOTHING = 0.5   # 0 = no smoothing (raw error), closer to 1 = heavier smoothing

MIN_MARK_LINE_DISTANCE = 40
MARK_CLEAR_FRAMES = 3

lower_bgr_values = np.array([100, 100, 100])
upper_bgr_values = np.array([200, 200, 200])

def crop_size(height, width):
    return (height//10, 5*height//6, width//8, 7*width//8)

image_input = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None
lost_frame_count = 0
mark_absent_count = 0


def start_follower_callback(request, response):
    global should_move, right_mark_count, finalization_countdown, lost_frame_count, mark_absent_count
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    lost_frame_count = 0
    mark_absent_count = 0
    return response

def stop_follower_callback(request, response):
    global should_move, finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    print(f"[DEBUG] image_callback: received frame shape={image_input.shape}", flush=True)

def get_contour_data(mask, out):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print(f"[DEBUG] get_contour_data: found {len(contours)} contours", flush=True)

    mark = {}
    line = {}

    line_candidates = []
    mark_candidates = []

    for contour in contours:
        M = cv2.moments(contour)
        area = M['m00']

        if area < MIN_AREA:
            continue

        cx = int(M['m10'] / area)
        cy = int(M['m01'] / area)

        cx_full = cx + crop_w_start

        if area > MAX_AREA_TRACK:
            continue

        if area >= MIN_AREA_TRACK:
            line_candidates.append((area, cx_full, cy))
            cv2.circle(out, (cx, cy), 5, (0, 255, 0), -1)
        else:
            mark_candidates.append((area, cx_full, cy))
            cv2.circle(out, (cx, cy), 5, (255, 0, 0), -1)
            print(f"[DEBUG] mark candidate: area={area:.1f} cx={cx_full} cy={cy}", flush=True)

    if line_candidates:
        _, cx_full, cy = max(line_candidates, key=lambda c: c[0])
        line['x'] = cx_full
        line['y'] = cy

    if mark_candidates:
        best_area, cx_full, cy = max(mark_candidates, key=lambda c: c[0])
        if not line or abs(cx_full - line.get('x', cx_full)) > MIN_MARK_LINE_DISTANCE:
            mark['x'] = cx_full
            mark['y'] = cy
            print(f"[DEBUG] mark WINNER: x={cx_full} y={cy} area={best_area:.1f} "
                  f"(dist_from_line={abs(cx_full - line.get('x', cx_full)) if line else 'N/A'})", flush=True)
        else:
            print(f"[DEBUG] mark candidate REJECTED (too close to line): x={cx_full} y={cy} "
                  f"area={best_area:.1f} line_x={line.get('x')}", flush=True)

    if mark and line:
        mark_side = "right" if mark['x'] > line['x'] else "left"
    else:
        mark_side = None

    return (line, mark_side)

def timer_callback():
    global error, image_input, just_seen_line, just_seen_right_mark
    global should_move, right_mark_count, finalization_countdown, lost_frame_count, mark_absent_count

    print(f"[DEBUG] timer_callback: fired, image_input_type={type(image_input).__name__}", flush=True)

    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape
    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)
    # Clean up the raw mask before contour detection: closing (dilate then
    # erode) merges small gaps/fragments in the line into one solid blob,
    # and removes isolated single-pixel noise, so the contour count and
    # line position stop flickering frame to frame.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cv2.imshow("mask", mask)
    colors, counts = np.unique(crop.reshape(-1, 3), axis=0, return_counts=True)
    top = np.argsort(-counts)[:8]
    print("Top colors in crop (BGR: count):", flush=True)
    for i in top:
        print(f"  {colors[i]}: {counts[i]}", flush=True)

    output = image
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])

    print(f"[DEBUG] line_detected={bool(line)} line={line} mark_side={mark_side}", flush=True)

    message = Twist()

    if line:
        raw_error = line['x'] - width / 2
        error = ERROR_SMOOTHING * error + (1 - ERROR_SMOOTHING) * raw_error
        just_seen_line = True
        lost_frame_count = 0
        message.linear.x = LINEAR_SPEED
    else:
        if just_seen_line:
            error = error * LOSS_FACTOR
            just_seen_line = False
        lost_frame_count += 1
        message.linear.x = LOST_LINEAR_SPEED

    print(f"[DEBUG] error={error:.2f} lost_frame_count={lost_frame_count} just_seen_line={just_seen_line}", flush=True)

    if mark_side == "right":
        mark_absent_count = 0
        if abs(error) < MAX_ERROR and not just_seen_right_mark:
            right_mark_count += 1
            just_seen_right_mark = True
            print(f"[DEBUG] RIGHT MARK COUNTED #{right_mark_count} | error={error:.2f} mark_side={mark_side}", flush=True)
            if right_mark_count >= 2 and finalization_countdown is None:
                finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD)
                print(f"[DEBUG] FINALIZATION ARMED at right_mark_count={right_mark_count}", flush=True)
    else:
        mark_absent_count += 1
        if mark_absent_count >= MARK_CLEAR_FRAMES:
            just_seen_right_mark = False

    if should_move:
        if lost_frame_count > SEARCH_TIMEOUT:
            message.linear.x = 0.0
            message.angular.z = -SEARCH_ANGULAR_SPEED if error > 0 else SEARCH_ANGULAR_SPEED
        else:
            message.angular.z = -error * KP
    else:
        message.linear.x = 0.0
        message.angular.z = 0.0

    print(f"[DEBUG] should_move={should_move} linear.x={message.linear.x:.3f} angular.z={message.angular.z:.3f}", flush=True)

    publisher.publish(message)

    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
    cv2.imshow("output", output)
    cv2.waitKey(5)

    if finalization_countdown is not None:
        if finalization_countdown > 0:
            finalization_countdown -= 1
        elif finalization_countdown == 0:
            should_move = False
            empty_message = Twist()
            publisher.publish(empty_message)
            print("Track Completed")
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()
            return


def main():
    print("[DEBUG] follower_node main() starting up", flush=True)
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
