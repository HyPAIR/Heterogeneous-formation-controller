#!/usr/bin/env python3

import rospy
from tf.msg import tfMessage


def main():
    rospy.init_node('tf_remove_child_frames')
    remove_frames = rospy.get_param('~remove_frames', ['base_link', 'base_footprint'])
    external_localization = rospy.get_param('~external_localization', False)
    
    # if internel localization is used, keep all frames
    if external_localization == False:
        remove_frames = []

    rospy.logerr('Removing frames: {}'.format(remove_frames))
    
    # filter tf_in topic
    tf_pub = rospy.Publisher('tf_out', tfMessage, queue_size=1)

    def tf_cb(msg):
        msg.transforms = [t for t in msg.transforms if t.child_frame_id not in remove_frames]
        if len(msg.transforms) > 0:
            tf_pub.publish(msg)

    rospy.Subscriber('tf_in', tfMessage, tf_cb)

    # filter tf_static_in topic
    tf_static_pub = rospy.Publisher('tf_static_out', tfMessage, queue_size=1, latch=True)

    def tf_static_cb(msg):
        msg.transforms = [t for t in msg.transforms if t.child_frame_id not in remove_frames]
        if len(msg.transforms) > 0:
            tf_static_pub.publish(msg)

    rospy.Subscriber('tf_static_in', tfMessage, tf_static_cb)

    rospy.spin()


if __name__ == '__main__':
    print('Starting tf_remove_child_frames node...')
    main()
