#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import tkinter as tk
import numpy as np
import time

# Variables globales para almacenar los valores de uMeas y wMeas
uMeas_value = 0.0
wMeas_value = 0.0

# Callbacks para actualizar los valores de uMeas y wMeas
def uMeas_callback(data):
    global uMeas_value
    uMeas_value = data.data

def wMeas_callback(data):
    global wMeas_value
    wMeas_value = data.data

def start_simulation():
    try:
        # Obtener los valores de la GUI
        tf = float(tf_entry.get())
        ts = float(ts_entry.get())
        #uRef_value = float(uRef_entry.get())
        #wRef_value = float(wRef_entry.get())
        hxd = float(hxd_entry.get()) # Posicion deseada en X
        hyd = float(hyd_entry.get()) # Posicion deseada en Y
        t = np.arange(0,tf+ts,ts) # vector tiempo

        N = len(t) # cantidad de muestras

        # Asignar memoria 
        x1 = np.zeros(N+1) 
        y1 = np.zeros(N+1)
        phi = np.zeros(N+1)

        hx = np.zeros(N+1) 
        hy = np.zeros(N+1)

        x1[0] = 0
        y1[0] = 0
        phi[0] = 0*(np.pi/180) # Orientacion inicial en radianes [rad]

        # Cinematica directa

        hx[0] = x1[0]+np.cos(phi[0]);  # Posicion inicial en el eje x en metros [m]
        hy[0] = y1[0]+np.sin(phi[0]);  # Posicion inicial en el eje y en metros [m]
        
        ################### VELOCIDADES DE REFERENCIA #################### 

        uRef = np.zeros(N)  # Velocidad lineal en metros/segundos [m/s]
        wRef = np.zeros(N) # Velocidad angular en radianes/segundos [rad/s]

        ################### VELOCIDADES MEDIDAS #################### 

        uMeas = np.zeros(N)  # Velocidad lineal en metros/segundos [m/s]
        wMeas = np.zeros(N)  # Velocidad angular en radianes/segundos [rad/s]

        ################### ERRORES ####################

        hxe = np.zeros(N) 
        hye = np.zeros(N)


        # Inicializar ROS
        rospy.init_node('cmd_vel_publisher', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Suscribirse a los tópicos de uMeas y wMeas
        rospy.Subscriber("uMeas", Float32, uMeas_callback)
        rospy.Subscriber("wMeas", Float32, wMeas_callback)

        # Esperar a que se establezca la conexión con el publicador
        rospy.sleep(1)

        # Crear el mensaje Twist
        twist_msg = Twist()

        for k in range(N):

            start_time = time.time() # Tiempo actual

            #################### CONTROL #####################

            # Errores
            hxe[k] = hxd - hx[k]
            hye[k] = hyd - hy[k]

            he = np.array([[hxe[k] ],[hye[k] ]])
            
            # Matriz Jacobiana
            J = np.array([[ np.cos(phi[k]), - np.sin(phi[k])],
                        [ np.sin(phi[k]),  np.cos(phi[k])]])

            # Parametros de control
            K = np.array([[ 0.18, 0],
                        [ 0,  0.18]])
            
            # Ley de control                   
            qpRef = np.linalg.pinv(J)@K@he

            #################### APLICAR ACCIONES DE CONTROL #####################

            uRef[k] = qpRef[0][0]
            wRef[k] = qpRef[1][0]

            # Asignar valores a las velocidades lineales y angulares del mensaje Twist
            twist_msg.linear.x = uRef[k]
            twist_msg.angular.z = wRef[k]

            # Publicar el mensaje en el tópico /cmd_vel
            cmd_vel_pub.publish(twist_msg)

            uMeas[k] = uMeas_value
            wMeas[k] = wMeas_value
            
            # Integral numerica
            phi[k+1] = phi[k]+ts*wMeas[k]

            # Modelo cinemático
            
            x1p = uMeas[k]*np.cos(phi[k+1])
            y1p = uMeas[k]*np.sin(phi[k+1])
            
            # Integral numerica
            x1[k+1] = x1[k] + ts*x1p
            y1[k+1] = y1[k] + ts*y1p
            
            # Cinematica directa     
            hx[k+1] = x1[k+1]+np.cos(phi[k+1]);  # Posicion inicial en el eje x en metros [m]
            hy[k+1] = y1[k+1]+np.sin(phi[k+1]);   # Posicion inicial en el eje y en metros [m]

            elapsed_time = time.time() - start_time # Tiempo transcurrido

            # Esperar el tiempo de muestreo
            rospy.sleep(ts-elapsed_time)

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

tk.Label(root, text="Posición deseada en X [m]:").grid(row=2, column=0)
hxd_entry = tk.Entry(root)
hxd_entry.grid(row=2, column=1)

tk.Label(root, text="Posición deseada en Y [m]::").grid(row=3, column=0)
hyd_entry = tk.Entry(root)
hyd_entry.grid(row=3, column=1)

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
