#!/usr/bin/env python3

import rospy
import json
import os
from std_msgs.msg import String
from characteristics_selector import choose_unique_categories

def main():
    rospy.init_node('json_tts_node', anonymous=True)
    tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)

    # Wait for subscribers to avoid missing the first message
    timeout = rospy.Time.now() + rospy.Duration(5.0)
    while tts_pub.get_num_connections() == 0 and rospy.Time.now() < timeout and not rospy.is_shutdown():
        rospy.loginfo("Waiting for subscriber to /text_to_speech...")
        rospy.sleep(0.1)

    # Load JSON
    script_dir = os.path.dirname(os.path.realpath(__file__))
    json_path = os.path.join(os.path.dirname(script_dir), "customers_database.json")
    rospy.loginfo("Reading JSON from: %s", json_path)

    with open(json_path, 'r') as f:
        data = json.load(f)

    # Use our imported greedy algorithm
    assignments = choose_unique_categories(data)

    # Publish results
    for person, char in assignments.items():
        msg = f"{person}: {char}"
        rospy.loginfo("Publishing: %s", msg)
        tts_pub.publish(String(msg))
        rospy.sleep(5.0)  # 2 seconds per person

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
