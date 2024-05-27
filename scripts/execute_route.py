'''
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(client, x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_state()

def execute_custom_route(coordinates):
    # Inicializar el nodo ROS y el cliente de acción
    rospy.init_node('custom_route_executor', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Seguir la ruta personalizada
    for point in coordinates:
        state = move_to_goal(client, point[0], point[1])
        if state != 3:  # 3 significa éxito en actionlib
            rospy.loginfo("Failed to reach the checkpoint.")
            break
        else:
            rospy.loginfo("Checkpoint alcanzado")

    rospy.loginfo("Ruta completada.")
'''
