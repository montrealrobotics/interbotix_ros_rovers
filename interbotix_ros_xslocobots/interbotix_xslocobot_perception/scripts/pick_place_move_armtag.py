import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import rospy
from geometry_msgs.msg import Twist
import random

# This script uses the perception pipeline to pick up objects and place them in some virtual basket on the left side of the robot
# The robot then makes a random move, either forward or makes a rotation clockwise or counter clockwise, these steps repeat.
# It also uses the AR tag on the arm to get a better idea of where the arm is relative to the camera (though the URDF is pretty accurate already).
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s use_perception:=true use_armtag:=true use_static_transform_pub:=true'
# Then change to this directory and type 'python pick_place_move_armtag.py'


def main():
    bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
    # Create a publisher which can "talk" to TurtleBot and tell it to move
    # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
    cmd_vel = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)

    # Twist is a datatype for velocity
    move_cmd = Twist()
    # let's go forward at 0.2 m/s
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0
    # move camera such that it's tilting down
    bot.camera.pan_tilt_move(0, 0.75)
    # position the arm such that the Apriltag is clearly visible to the camera
    bot.arm.set_ee_pose_components(x=0.2, z=0.2, pitch=-math.pi/8.0)
    # sleep half a second to give time for the arm to settle after moving
    time.sleep(0.5)
    # get the transform of the AR tag relative to the camera frame; based on that transform,
    # publish a new transform from the 'locobot/plate_link' to the 'locobot/arm_base_link'
    bot.armtag.find_ref_to_arm_base_transform(position_only=True)
    # move the arm out of the way of the camera
    bot.arm.go_to_sleep_pose()
    # get the positions of any clusters present w.r.t. the 'locobot/arm_base_link';
    # sort the clusters such that they appear from left-to-right w.r.t. the 'locobot/arm_base_link'
    time.sleep(0.5)
    last_clusters = []
    clusters = []
    try:
        while True:
            if clusters:
                # save all seen clusters so that we don't attempt to pick ghost clusters - a hack
                last_clusters.append(clusters[0])

            success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y",
                                                              reverse=True)

            # move the robot back so it's centered and open the gripper
            if success:
                for cluster in clusters:
                    x, y, z = cluster["position"]
                    for old_cluster in last_clusters:
                        x_old, y_old, z_old = old_cluster["position"]
                        if math.isclose(x, x_old, abs_tol=0.05):
                            # remove cluster if a duplicate - means that the point cloud hasn't been updated.
                            clusters.remove(cluster)
                            break
                if clusters:
                    # only move arm into ready position if there are clusters to pick
                    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
                    bot.gripper.open()

            # pick up each object from left-to-right and drop it in a virtual basket on the left side of the robot
            for cluster in clusters:
                x, y, z = cluster["position"]
                bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
                bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
                bot.gripper.close()
                bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
                bot.arm.set_ee_pose_components(y=0.3, z=0.2)
                bot.gripper.open()
                bot.arm.set_ee_pose_components(x=0.3, z=0.2)
            bot.arm.go_to_sleep_pose()

            # Generate a random float for velocity magnitude
            rand_float = random.uniform(0.0, 0.5)
            # generate a random int to select motion type.
            selection = random.randint(0, 3)

            if selection == 0:
                move_cmd.linear.x = rand_float
                move_cmd.angular.z = 0
            elif selection == 1:
                move_cmd.angular.z = rand_float * 10
                move_cmd.linear.x = 0
            else:
                move_cmd.angular.z = -rand_float * 10
                move_cmd.linear.x = 0
            # publish the velocity
            cmd_vel.publish(move_cmd)
            rospy.sleep(1)

    except KeyboardInterrupt:
        print('Pick and Place interrupted!')


if __name__=='__main__':
    main()
