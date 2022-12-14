import numpy as np
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from uav_pid_control.msg import PositionTarget

PATH = np.array(
    [
        [0.0, 0.0, 10.0],
        [14.14213562373095, 14.14213562373095, 10.0],
        [0.0, 28.2842712474619, 10.0],
        [-14.14213562373095, 14.14213562373095, 10.0],
        [0.0, 0.0, 10.0],
    ]
)


def main():
    rospy.init_node("demo_uav_pid_control")
    ros_objects = {}
    vehicle_position = np.zeros((3,))

    def pose_cb(msg):
        for idx, it in enumerate("xyz"):
            vehicle_position[idx] = getattr(msg.pose.position, it)

    ros_objects["pose_sub"] = rospy.Subscriber(
        "/mavros/local_position/pose", PoseStamped, pose_cb, queue_size=1
    )

    ros_objects["setpoint_pub"] = rospy.Publisher(
        "/uav_pid_control/setpoints/position", PositionTarget, queue_size=1
    )

    def main_loop(timer_event):
        delta = vehicle_position - PATH[main_loop.wp_idx, :]
        if delta @ delta < 1.0:
            rospy.loginfo("Reached waypoint %d" % main_loop.wp_idx)
            rospy.sleep(5.0)
            main_loop.wp_idx += 1
        try:
            setpoint = PATH[main_loop.wp_idx, :]
            if main_loop.wp_idx > 0:
                delta_wp = setpoint - PATH[main_loop.wp_idx - 1, :]
                heading = np.arctan2(delta_wp[1], delta_wp[0])
            else:
                heading = 0.0
            pld = PositionTarget(
                header=Header(stamp=timer_event.current_real),
                position=Point(x=setpoint[0], y=setpoint[1], z=setpoint[2]),
                heading=heading,
            )
            ros_objects["setpoint_pub"].publish(pld)
        except IndexError:
            ros_objects["main_loop"].shutdown()

    main_loop.wp_idx = 0
    ros_objects["main_loop"] = rospy.Timer(rospy.Duration.from_sec(0.1), main_loop)
    rospy.spin()


if __name__ == "__main__":
    main()
