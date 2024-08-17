import rospy
from geometry_msgs.msg import Twist

def main():
    # Inicializar el nodo ROS
    rospy.init_node('cmd_vel_publisher')

    # Crear un publicador para el tópico /cmd_vel
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Esperar a que se establezca la conexión con el publicador
    rospy.sleep(1)

    # Parámetros de entrada para la velocidad lineal y angular
    tf = 10#float(input("Ingrese el tiempo de simulación en segundos (tf): "))
    ts = 0.1#float(input("Ingrese el tiempo de muestreo en segundos (ts): "))
    uRef_value = 0.5#float(input("Ingrese la velocidad lineal de referencia en metros/segundos (uRef): "))
    wRef_value = 0.0#float(input("Ingrese la velocidad angular de referencia en radianes/segundos (wRef): "))

    # Calcular la cantidad de muestras
    t = int(tf/ts)
    
    # Crear el mensaje Twist para enviar los valores de uRef y wRef
    twist_msg = Twist()

    for k in range(t):
        # Asignar valores a las velocidades lineales y angulares del mensaje Twist
        twist_msg.linear.x = uRef_value
        twist_msg.angular.z = wRef_value

        # Publicar el mensaje en el tópico /cmd_vel
        cmd_vel_pub.publish(twist_msg)

        # Esperar el tiempo de muestreo
        rospy.sleep(ts)

    # Detener el robot después de finalizar la simulación
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)

    rospy.loginfo("Simulación completada y robot detenido.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
