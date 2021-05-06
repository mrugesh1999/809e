#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist, Point, PoseStamped


def marker_map(msg):
    """
    Send the transform to the frame named /follower_tf/camera_rgb_optical_frame
    :param msg: The message published on the topic
    :return: None
    """
    if msg.transforms:
        data = msg.transforms[0].transform
        br = tf.TransformBroadcaster()

        br.sendTransform((data.translation.x, data.translation.y, data.translation.z - 0.7),
                         tf.transformations.quaternion_from_euler(0, 0, data.rotation.w),
                         rospy.Time.now(),
                         "follower_tf/marker_to_camera",
                         "follower_tf/camera_rgb_optical_frame")


def get_marker_map(tf_listen):
    """
    Send the final transform and bool of marker is_detected
    :param tf_listen: tf listener instance
    :return: The transform and bool if marker exists
    """
    try:
        transform = tf_listen.lookupTransform('follower_tf/odom', 'follower_tf/marker_to_camera', rospy.Time(0))
        return transform, True
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        return None, False


def pose():
    """Gets the leader turtlebot's current pose using a transform between odom and base_footprint
    return: the current position and orientation of the leader turtlebot
    """
    try:
        (trans, rot) = tf_listener.lookupTransform('/leader_tf/odom', '/leader_tf/base_footprint', rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    return Point(*trans), rotation[2]


def movebase_client(x, y, w):
    """
        Move_base_simple takes the follower robot to given position
        :param x: Goal Point x
        :param y: Goal Point y
        :param w: Goal Point w (orientation)
        :return: None
        """
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.w = w
    rate = rospy.Rate(1)
    pub2.publish(message)
    rate.sleep()
    (position, rotation) = pose()
    x_current = position.x
    y_current = position.y
    w_current = rotation
    if x - 0.5 < x_current < x + 0.5 and y - 0.5 < y_current < y + 0.5 and w - 20 < w_current < w + 20:
        pass


def rotate(publish, theta):
    """
    Rotate the follower
    :param publish: The publisher for publishing Twist msg
    :param theta: Angular velocity
    :return: Null
    """
    velocity = Twist()
    velocity.angular.z = theta
    rate = rospy.Rate(1)
    publish.publish(velocity)
    rate.sleep()


# Initiate the nodes and tf listeners
tf_listener = tf.TransformListener()
rospy.init_node("follower", anonymous=True)
rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, marker_map)
rospy.sleep(1)
pub = rospy.Publisher("/follower/cmd_vel", Twist, queue_size=5)
pub2 = rospy.Publisher("/follower/move_base_simple/goal", PoseStamped, queue_size=10)
tf_listener = tf.TransformListener()

# Initiating the variables
state = None
rospy.sleep(1)
rotate(pub, 0)

# Checking if the transform is available
try:
    tf_listener.waitForTransform('/follower_tf/odom', '/follower_tf/base_footprint', rospy.Time(), rospy.Duration(1))
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    rospy.loginfo(
        "Cannot find transform between {p} and {c}".format(p='/follower_tf/odom', c='/follower_tf/base_footprint'))
    rospy.signal_shutdown("tf Exception")

# Get the current pose of the follower
(follower_location, _) = pose()

# Getting the marker pose
marker_location, is_marker = get_marker_map(tf_listener)
exploration = True

# Looping forever
while True:

    # Getting the marker pose
    marker_location, is_marker = get_marker_map(tf_listener)

    # If the marker is found
    if is_marker:

        # Making sure the print is executed once
        if state != "Following":
            print("Marker Found")
            print("Following Marker")
            state = "Following"

        # Setting the status variable
        exploration = True

        # Commending follower to go to marker location
        movebase_client(marker_location[0][0], marker_location[0][1], marker_location[0][2])

        # Getting current pose of follower
        (follower_location, _) = pose()

    # If exploration has to be done
    elif exploration:

        # Making sure it prints only once
        if state != "Rotating":
            print("Looking for Marker: rotating")
            state = "Rotating"

        # Asking it to rotate CCW
        rotate(pub, 1.5)
        time_now = rospy.Time.now()

        # Rotating till four seconds or till marking
        while rospy.Time.now().secs - time_now.secs < 4:
            marker_location, is_marker = get_marker_map(tf_listener)
            if is_marker:

                # If the marker is found stop rotating and exit the loop
                rotate(pub, 0)
                break
        else:

            # If the marker is still not found just execute next subroutine
            rotate(pub, 0)
            exploration = False
    else:

        # Printing the exploring only once
        if state != "Exploring":
            print("Looking for Marker: exploring")
            state = "Exploring"
        try:

            # Go to the location of leader goal
            next_goal = rospy.get_param('leader_goal')
            movebase_client(next_goal[0] - 0.7, next_goal[1], -float(next_goal[2]))
            (follower_location, _) = pose()
        except KeyError:
            pass
