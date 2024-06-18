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

def follow_route(route_number):
    # Define las rutas con listas de coordenadas
    # [x, y, Rz, parada]
    route_1 = [
        [-1.829587459564209, -0.4259052276611328, 0, False],
        [-2.416154146194458, -4.989470481872559, 1,57, True],
        [-0.9932656288146973, -9.363847732543945, 0, True],
        [-5.4887895584106445, -9.122173309326172, 3.14, True],
        [-10.311999320983887, -9.096759796142578, 0, True]
    ]
    route_2 = [
        [1.0, 1.0, 0, True],
        [2.0, 2.0, 0, True],
        [3.0, 3.0, 0, True]
    ]
    route_3 = [
        [4.0, 4.0, 0, True],
        [5.0, 5.0, 0, True],
        [6.0, 6.0, 0, True]
    ]
    route_4 = [
        [7.0, 7.0, 0, True],
        [8.0, 8.0, 0, True],
        [9.0, 9.0, 0, True]
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

    # Inicializa el cliente de acción
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Sigue la ruta
    for point in route:
        state = move_to_goal(client, point[0], point[1], point[2])
        print(state)
        if state != 3:  # 3 significa éxito en actionlib
            rospy.loginfo("Failed to reach the checkpoint.")
            break
        else:
            print("Checkpoint alcanzado")
            # para usar puntos intermedios y que no haga paradas en esos puntos
            if point[3]:
                time.sleep(5)

    rospy.loginfo("Ruta completada.")