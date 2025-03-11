#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
import numpy as np

# Inicializar la aplicación de PyQtGraph
app = QtWidgets.QApplication([])

# Crear una ventana para la gráfica
win = pg.GraphicsLayoutWidget(show=True, title="Realtime EMG Plot")
plot = win.addPlot(title="EMG Signal")
curve = plot.plot(pen='y')  # Línea amarilla

# Configuración de la gráfica
plot.setLabel('left', 'Voltage', units='V')
plot.setLabel('bottom', 'Samples')
plot.setYRange(-1000, 1000)  # Rango de voltaje típico para sensores EMG
data = []

# Callback para recibir datos de ROS
def callback(msg):
    global data
    data.append(msg.data)
    if len(data) > 1000:  # Limitar la cantidad de puntos en la gráfica
        data.pop(0)

# Inicializar nodo de ROS
rospy.init_node("emg_plotter", anonymous=True)
rospy.Subscriber("/emg_signal", Float32, callback)

# Función de actualización de la gráfica
def update():
    if data:  # Solo actualiza si hay datos
        curve.setData(data)

# Crear un temporizador para actualizar la gráfica cada 50 ms
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # 50 ms = 20 Hz

# Ejecutar la aplicación de PyQtGraph
QtWidgets.QApplication.instance().exec_()
