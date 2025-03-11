#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32
import csv
import os
from datetime import datetime

# Configuración del puerto serie
SERIAL_PORT = "/dev/cu.usbserial-110"
BAUD_RATE = 115200

# Directorio y archivo para guardar los datos
output_dir = "emg_data"
os.makedirs(output_dir, exist_ok=True)

# Nombre del archivo con timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_file = os.path.join(output_dir, f"emg_{timestamp}.csv")

def read_emg():
    rospy.init_node("emg_publisher", anonymous=True)
    pub = rospy.Publisher("emg_signal", Float32, queue_size=10)
    sample_id = 0  # Contador de muestras

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        rospy.loginfo(f"Conectado a {SERIAL_PORT} a {BAUD_RATE} baudios")
        
        with open(output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Sample_ID", "Timestamp", "EMG_Raw"])  # Cabecera del CSV
            
            while not rospy.is_shutdown():
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        emg_value = float(line)  # Valor EMG en bruto
                        pub.publish(emg_value)

                        # Guardar datos con timestamp en ISO 8601
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        writer.writerow([sample_id, timestamp, emg_value])
                        sample_id += 1

                        rospy.loginfo(f"[{sample_id}] {timestamp} | EMG: {emg_value:.3f}")
                except ValueError:
                    rospy.logwarn("Error al convertir el valor leído")

    except serial.SerialException as e:
        rospy.logerr(f"No se pudo abrir el puerto serial: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    read_emg()
