#!/usr/bin/env python3
"""Publish one Boreas Applanix odometry message for Kimera GT initialization."""

import csv
import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_matrix


def roll(r):
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, math.cos(r), math.sin(r)], [0.0, -math.sin(r), math.cos(r)]],
        dtype=np.float64,
    )


def pitch(p):
    return np.array(
        [[math.cos(p), 0.0, -math.sin(p)], [0.0, 1.0, 0.0], [math.sin(p), 0.0, math.cos(p)]],
        dtype=np.float64,
    )


def yaw(y):
    return np.array(
        [[math.cos(y), math.sin(y), 0.0], [-math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def yaw_pitch_roll_to_rot(y, p, r):
    return roll(r) @ pitch(p) @ yaw(y)


def load_first_row_at_or_after(gps_csv, start_sec):
    with open(gps_csv, "r", encoding="utf-8", errors="replace", newline="") as stream:
        reader = csv.DictReader(stream)
        fallback = None
        for row in reader:
            fallback = row
            if float(row["GPSTime"]) >= start_sec:
                return row
        if fallback is None:
            raise RuntimeError("empty Boreas gps_post_process.csv")
        return fallback


def row_to_odometry(row, frame_id, child_frame_id):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = yaw_pitch_roll_to_rot(
        float(row["heading"]),
        float(row["pitch"]),
        float(row["roll"]),
    )
    transform[:3, 3] = [
        float(row["easting"]),
        float(row["northing"]),
        float(row["altitude"]),
    ]
    qx, qy, qz, qw = quaternion_from_matrix(transform)
    world_v_body = np.array(
        [float(row["vel_east"]), float(row["vel_north"]), float(row["vel_up"])],
        dtype=np.float64,
    )
    body_v_body = transform[:3, :3].T @ world_v_body

    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(float(row["GPSTime"]))
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(transform[0, 3])
    msg.pose.pose.position.y = float(transform[1, 3])
    msg.pose.pose.position.z = float(transform[2, 3])
    msg.pose.pose.orientation.x = float(qx)
    msg.pose.pose.orientation.y = float(qy)
    msg.pose.pose.orientation.z = float(qz)
    msg.pose.pose.orientation.w = float(qw)
    msg.twist.twist.linear.x = float(body_v_body[0])
    msg.twist.twist.linear.y = float(body_v_body[1])
    msg.twist.twist.linear.z = float(body_v_body[2])
    return msg


def main():
    rospy.init_node("boreas_gt_init_publisher")
    gps_csv = rospy.get_param("~gps_csv")
    start_sec = float(rospy.get_param("~start_sec", 0.0))
    topic = rospy.get_param("~topic", "/boreas/gt_odom_init")
    frame_id = rospy.get_param("~frame_id", "map")
    child_frame_id = rospy.get_param("~child_frame_id", "imu_link")
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
    repeat_count = int(rospy.get_param("~repeat_count", 200))

    row = load_first_row_at_or_after(gps_csv, start_sec)
    msg = row_to_odometry(row, frame_id, child_frame_id)
    pub = rospy.Publisher(topic, Odometry, queue_size=1, latch=True)
    rate = rospy.Rate(publish_rate_hz)

    while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0.0:
        rate.sleep()

    for _ in range(max(1, repeat_count)):
        if rospy.is_shutdown():
            break
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo(
        "Published Boreas GT init odometry at %.6f to %s",
        msg.header.stamp.to_sec(),
        topic,
    )


if __name__ == "__main__":
    main()
