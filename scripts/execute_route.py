import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import time

def move_to_goal(client, x, y, rz):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quaternion = quaternion_from_euler(0, 0, rz)
    goal.target_pose.pose.orientation = Quaternion(*quaternion)
    
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_state()

def execute_custom_route(coordinates):
    rospy.init_node('custom_route_executor', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    for point in coordinates:
        state = move_to_goal(client, point[0], point[1], point[2])
        if state != 3:  # 3 significa éxito en actionlib
            rospy.loginfo("Failed to reach the checkpoint.")
            break
        else:
            rospy.loginfo("Checkpoint alcanzado")
            if point[3]:  # Si es una parada
                time.sleep(5)

    rospy.loginfo("Ruta completada.")
