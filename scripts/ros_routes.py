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

def follow_route(route_number):
    # Define las rutas con listas de coordenadas
    route_1 = [
        [156.11671447753906, 53.39218521118164],
        [155.8867645263672, 49.3564453125],
        [151.76332092285156, 46.81568908691406],
        [155.70179748535156, 41.81671905517578],
        [159.77371215820312, 35.37627410888672],
        [154.77906799316406, 33.564361572265625]
    ]
    route_2 = [
        [1.0, 1.0],
        [2.0, 2.0],
        [3.0, 3.0]
    ]
    route_3 = [
        [4.0, 4.0],
        [5.0, 5.0],
        [6.0, 6.0]
    ]
    route_4 = [
        [7.0, 7.0],
        [8.0, 8.0],
        [9.0, 9.0]
    ]

    # Mapa de rutas
    routes = {
        1: route_1,
        2: route_2,
        3: route_3,
        4: route_4
    }

    # Verifica que el número de ruta sea válido
    if route_number not in routes:
        print("Número de ruta inválido")
        return

    # Obtiene la ruta correspondiente
    route = routes[route_number]

    # Inicializa el nodo ROS y el cliente de acción
    rospy.init_node('route_follower', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Sigue la ruta
    for point in route:
        state = move_to_goal(client, point[0], point[1])
        print(state)
        if state != 3:  # 3 significa éxito en actionlib
            rospy.loginfo("Failed to reach the checkpoint.")
            break
        else:
            print("Checkpoint alcanzado")

    rospy.loginfo("Ruta completada.")
'''