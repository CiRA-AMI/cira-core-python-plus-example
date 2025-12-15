#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CiRA CORE - ROS Interface Template (Python / ROS1)

- Service: /cira_digital_io/digital_output_cmd  (cira_msgs/CiraStrCmdService)
- Service: /cira_io/status                     (cira_msgs/CiraStrCmdService)
- Parse string format: "key=val,key2=val2,_extra=1"
- Print debug only (no real HW logic)

- Background thread publishes a solid-color image (640x480) every 1 second
- Image topic: /cira_ros_img
- Using ros_numpy (NO cv_bridge)
"""

import rospy
import threading
import numpy as np
import json
import random
import ros_numpy

from sensor_msgs.msg import Image

# IMPORTANT: this must match your C++ code service type
from cira_msgs.srv import CiraStrCmdService, CiraStrCmdServiceResponse


# ============================================================
# Utility: parse string command
# ============================================================
def parse_str_cmd(cmd_str):
    """
    Parse string command into dictionaries.

    Input example:
        "LED=1,SERVO1=90,_mode=2"

    Returns:
        cmds        : normal commands (dict)
        extra_cmds  : commands starting with '_' (dict)
    """
    rospy.loginfo(f"[DEBUG] raw input: '{cmd_str}'")

    cmds = {}
    extra_cmds = {}

    if not cmd_str:
        rospy.logwarn("[DEBUG] empty command string")
        return cmds, extra_cmds

    parts = [p.strip() for p in cmd_str.split(",") if p.strip()]
    rospy.loginfo(f"[DEBUG] split parts: {parts}")

    for p in parts:
        if "=" not in p:
            rospy.logwarn(f"[WARN] skip invalid token: {p}")
            continue

        name, val_str = p.split("=", 1)
        name = name.strip()
        val_str = val_str.strip()

        try:
            val = int(val_str)
        except ValueError:
            rospy.logwarn(f"[WARN] skip invalid value (not int): {p}")
            continue

        if name.startswith("_"):
            extra_cmds[name] = val
        else:
            cmds[name] = val

        rospy.loginfo(f"[DEBUG] parsed -> {name} = {val}")

    rospy.loginfo(f"[DEBUG] cmds       : {cmds}")
    rospy.loginfo(f"[DEBUG] extra_cmds : {extra_cmds}")

    return cmds, extra_cmds


# ============================================================
# Service: digital output command (string in, string out)
# ============================================================
def process_str_cmd_srv(req):
    """
    Mimics RosInterface::processStrCmd()

    Expected request fields (based on your C++):
      - req.str_in

    Response:
      - res.str_out
    """
    rospy.loginfo("[SERVICE] /cira_digital_io/digital_output_cmd called")

    cmd_str = req.str_in
    cmds, extra_cmds = parse_str_cmd(cmd_str)

    if not cmds and not extra_cmds:
        return CiraStrCmdServiceResponse(str_out="Error : wrong command")

    # -------- USER IMPLEMENTATION AREA --------
    rospy.loginfo("[INFO] TODO: implement hardware mapping here")

    for k, v in cmds.items():
        rospy.loginfo(f"[HW] normal cmd -> {k} = {v}")

    for k, v in extra_cmds.items():
        rospy.loginfo(f"[HW] extra cmd  -> {k} = {v}")
    # -----------------------------------------

    return CiraStrCmdServiceResponse(str_out="OK")


# ============================================================
# Service: get IO status (string in, string out)
# ============================================================
def process_get_io_srv(req):
    """
    Mimics RosInterface::processGetIO()

    Your C++ expects:
      req.str_in == "getIO"

    Returns:
      res.str_out as JSON string
    """
    rospy.loginfo("[SERVICE] /cira_io/status called")

    if req.str_in != "getIO":
        return CiraStrCmdServiceResponse(str_out="Wrong command")

    # -------- USER IMPLEMENTATION AREA --------
    # Build JSON-style IO status
    status = {
        "some_input": random.randint(0, 100)
    }

    json_str = json.dumps(status)
    rospy.loginfo(f"[DEBUG] IO status JSON: {json_str}")
    # -----------------------------------------

    return CiraStrCmdServiceResponse(str_out=json_str)


# ============================================================
# Image publisher thread (solid color)
# ============================================================
class ImagePublisher(threading.Thread):
    """
    Publish a solid-color BGR image (640x480) every 1 second.
    Using ros_numpy (NO cv_bridge).
    """

    def __init__(self):
        super(ImagePublisher, self).__init__()
        self.daemon = True

        self.width = 640
        self.height = 480
        self.encoding = "bgr8"

        self.pub = rospy.Publisher("/cira_ros_img", Image, queue_size=1)

    def run(self):
        rospy.loginfo("[THREAD] ImagePublisher started (solid color, ros_numpy)")

        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():

            # Generate ONE random BGR color for the whole image
            color = np.random.randint(0, 256, (3,), dtype=np.uint8)

            # Fill entire image with that color
            img = np.full((self.height, self.width, 3), color, dtype=np.uint8)

            msg = ros_numpy.msgify(Image, img, encoding=self.encoding)
            msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "cira_cam"  # optional

            self.pub.publish(msg)
            rospy.loginfo(f"[IMG] Published solid color image 640x480, color(BGR)={color.tolist()}")

            rate.sleep()


# ============================================================
# ROS Node main
# ============================================================
def main():
    rospy.init_node("cira_ros_interface_template")
    rospy.loginfo("[INIT] CiRA ROS Interface Template (service MD5 must match C++ type)")

    # Services (MUST match client type: cira_msgs/CiraStrCmdService)
    rospy.Service("/cira_digital_io/digital_output_cmd", CiraStrCmdService, process_str_cmd_srv)
    rospy.Service("/cira_io/status", CiraStrCmdService, process_get_io_srv)

    rospy.loginfo("[READY] Services created:")
    rospy.loginfo("  - /cira_digital_io/digital_output_cmd  (cira_msgs/CiraStrCmdService)")
    rospy.loginfo("  - /cira_io/status                     (cira_msgs/CiraStrCmdService)")
    rospy.loginfo("[READY] Image topic: /cira_ros_img")

    # Start image thread
    ImagePublisher().start()

    rospy.spin()


if __name__ == "__main__":
    main()
