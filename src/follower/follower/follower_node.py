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

# --- Values below are confirmed against actual sampled data from this sim:
# track line reads ~120-125 gray, floor reads ~0 in some spots and ~190-196
# in others, a separate white marking reads 255. ---
MIN_AREA = 150
MIN_AREA_TRACK = 300
MAX_AREA_TRACK = 10000   # excludes floor patches that happen to fall in the color range
LINEAR_SPEED = 0.2
KP = 1.5/100
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.06
MAX_ERROR = 30
SEARCH_TIMEOUT = int(1.0 / TIMER_PERIOD)
SEARCH_ANGULAR_SPEED = 0.5
LOST_LINEAR_SPEED = LINEAR_SPEED * 0.4

# --- Lap completion via odometry, since this track has no distinct marker
# objects to detect visually. A lap is "complete" once the robot has
# travelled far enough away from its start point (so we don't trigger
# immediately at spawn) and then come back close to that same point. ---
MIN_DISTANCE_AWAY = 1.0     # meters the robot must get away from start before "return" can count
RETURN_RADIUS = 0.3         # meters - close enough to start to count as "returned"

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

# --- Loop-lock detection ---
# Even with predictive matching, a self-connecting loop can still look
# like a "smooth continuation" every time through it. As a backstop, we
# watch total distance driven (via odom) vs. how far the robot actually
# ranges from a recent reference point. If it drives a lot of distance
# while staying spatially confined, it's circling the same loop, and we
# force a branch switch to break out.
LOOP_CHECK_DISTANCE = 3.0   # meters driven before checking for loop-lock
LOOP_RADIUS = 0.5           # meters - staying within this range over that driven distance means "stuck circling"
odom_history_dist = 0.0
loop_ref_x = None
loop_ref_y = None
force_branch_switch = False


def start_follower_callback(request, response):
    global should_move, lost_frame_count, last_line_pos, last_line_vel
    global start_x, start_y, has_traveled_away, lap_completing
    global odom_history_dist, loop_ref_x, loop_ref_y, force_branch_switch
    should_move = True
    lost_frame_count = 0
    last_line_pos = None
    last_line_vel = (0, 0)
    # Record the current position as the lap's start/finish point.
    start_x = current_x
    start_y = current_y
    has_traveled_away = False
    lap_completing = False
    odom_history_dist = 0.0
    loop_ref_x = current_x
    loop_ref_y = current_y
    force_branch_switch = False
    return response

def stop_follower_callback(request, response):
    global should_move
    should_move = False
    return response

def image_callback(msg):
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def odom_callback(msg):
    global current_x, current_y, odom_history_dist
    new_x = msg.pose.pose.position.x
    new_y = msg.pose.pose.position.y
    if current_x is not None:
        odom_history_dist += math.hypot(new_x - current_x, new_y - current_y)
    current_x = new_x
    current_y = new_y

def get_contour_data(mask, out):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print(f"[DEBUG] found {len(contours)} contours", flush=True)

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

    if line_candidates:
        global last_line_pos, last_line_vel, force_branch_switch
        if last_line_pos is not None:
            pred_x = last_line_pos[0] + last_line_vel[0]
            pred_y = last_line_pos[1] + last_line_vel[1]

            if force_branch_switch and len(line_candidates) > 1:
                # Loop-lock backstop fired: deliberately pick the
                # candidate FARTHEST from the predicted continuation,
                # forcing a switch away from whatever branch we've been
                # circling.
                _, cx_full, cy = max(
                    line_candidates,
                    key=lambda c: math.hypot(c[1] - pred_x, c[2] - pred_y)
                )
                force_branch_switch = False
                print(f"[DEBUG] LOOP BREAK: forcing switch to x={cx_full} y={cy}", flush=True)
            else:
                # Normal case: match whichever candidate best continues
                # the trajectory we were already on. This distinguishes
                # branches at a crossing better than raw last-position
                # proximity, since two crossing lines have different
                # tangent directions even where they're spatially close.
                _, cx_full, cy = min(
                    line_candidates,
                    key=lambda c: math.hypot(c[1] - pred_x, c[2] - pred_y)
                )
        else:
            _, cx_full, cy = max(line_candidates, key=lambda c: c[0])

        if last_line_pos is not None:
            last_line_vel = (cx_full - last_line_pos[0], cy - last_line_pos[1])
        last_line_pos = (cx_full, cy)
        line['x'] = cx_full
        line['y'] = cy

    return line

def timer_callback():
    global error, image_input, just_seen_line
    global should_move, lost_frame_count
    global has_traveled_away, lap_completing

    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape
    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)
    cv2.imshow("mask", mask)

    output = image
    line = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])

    print(f"[DEBUG] line_detected={bool(line)} line={line}", flush=True)

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

    print(f"[DEBUG] error={error:.2f}", flush=True)

    # --- Lap completion check (odometry-based) ---
    if should_move and start_x is not None and current_x is not None and not lap_completing:
        dist_from_start = math.hypot(current_x - start_x, current_y - start_y)
        if not has_traveled_away:
            if dist_from_start > MIN_DISTANCE_AWAY:
                has_traveled_away = True
                print(f"[DEBUG] traveled away from start (dist={dist_from_start:.2f}m) - return can now count", flush=True)
        else:
            if dist_from_start < RETURN_RADIUS:
                lap_completing = True
                print(f"[DEBUG] LAP COMPLETE - returned to start (dist={dist_from_start:.2f}m)", flush=True)

    # --- Loop-lock check (odometry-based) ---
    # If we've driven a lot of distance but stayed within a small radius
    # of a reference point, we're circling the same loop over and over.
    # Force a branch switch and reset the reference so we don't
    # immediately re-trigger.
    global odom_history_dist, loop_ref_x, loop_ref_y, force_branch_switch
    if should_move and current_x is not None and loop_ref_x is not None:
        dist_from_ref = math.hypot(current_x - loop_ref_x, current_y - loop_ref_y)
        if odom_history_dist > LOOP_CHECK_DISTANCE:
            if dist_from_ref < LOOP_RADIUS:
                force_branch_switch = True
                print(f"[DEBUG] LOOP-LOCK DETECTED: drove {odom_history_dist:.2f}m but only "
                      f"{dist_from_ref:.2f}m from reference point - forcing branch switch", flush=True)
            # Reset the odometer/reference regardless, so we're always
            # checking distance traveled since the last checkpoint.
            odom_history_dist = 0.0
            loop_ref_x, loop_ref_y = current_x, current_y

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

    print(f"[DEBUG] should_move={should_move} linear.x={message.linear.x:.3f} angular.z={message.angular.z:.3f}", flush=True)

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
    subscription = node.create_subscription(Image, 'camera/image_raw',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)
    odom_subscription = node.create_subscription(Odometry, '/odom',
                                                  odom_callback,
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
