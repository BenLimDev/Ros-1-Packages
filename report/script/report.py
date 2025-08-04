#!/usr/bin/env python3

import rospy
import json
import os
from std_msgs.msg import String
from characteristics_selector import choose_unique_categories  # updated function name

def main():
    rospy.init_node('report_node', anonymous=True)
    tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)

    timeout = rospy.Time.now() + rospy.Duration(5.0)
    while tts_pub.get_num_connections() == 0 and rospy.Time.now() < timeout and not rospy.is_shutdown():
        rospy.loginfo("Waiting for subscriber to /text_to_speech...")
        rospy.sleep(0.1)

    json_path = rospy.get_param("~json_file", None)
    if not json_path or not os.path.exists(json_path):
        rospy.logerr(f"JSON file not found or param not set: {json_path}")
        return

    rospy.loginfo(f"Reading JSON from: {json_path}")
    with open(json_path, 'r') as f:
        data = json.load(f)

    assignments = choose_unique_categories(data)

    for sentence in assignments.values():
        rospy.loginfo("Publishing: %s", sentence)
        tts_pub.publish(String(sentence))
        rospy.sleep(5.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
