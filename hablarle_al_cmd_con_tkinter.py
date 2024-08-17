#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tkinter as tk

def start_simulation():
    try:
        # Obtener los valores de la GUI
        tf = float(tf_entry.get())
        ts = float(ts_entry.get())
        uRef_value = float(uRef_entry.get())
        wRef_value = float(wRef_entry.get())

        # Calcular la cantidad de muestras
        t = int(tf/ts)

        # Inicializar ROS
        rospy.init_node('cmd_vel_publisher', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Esperar a que se establezca la conexión con el publicador
        rospy.sleep(1)

        # Crear el mensaje Twist
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
    except ValueError:
        rospy.loginfo("Introduce valores numéricos válidos")

# Crear la ventana principal
root = tk.Tk()
root.title("Control de Robot - GUI")

# Crear las entradas para los parámetros
tk.Label(root, text="Tiempo de simulación [s]:").grid(row=0, column=0)
tf_entry = tk.Entry(root)
tf_entry.grid(row=0, column=1)

tk.Label(root, text="Tiempo de muestreo [s]:").grid(row=1, column=0)
ts_entry = tk.Entry(root)
ts_entry.grid(row=1, column=1)

tk.Label(root, text="Velocidad lineal [m/s]:").grid(row=2, column=0)
uRef_entry = tk.Entry(root)
uRef_entry.grid(row=2, column=1)

tk.Label(root, text="Velocidad angular [rad/s]:").grid(row=3, column=0)
wRef_entry = tk.Entry(root)
wRef_entry.grid(row=3, column=1)

# Botón para iniciar la simulación
start_button = tk.Button(root, text="Iniciar Simulación", command=start_simulation)
start_button.grid(row=4, column=0, columnspan=2)

# Iniciar la GUI
root.mainloop()

if __name__ == '__main__':
    try:
        start_simulation()
    except rospy.ROSInterruptException:
        pass
