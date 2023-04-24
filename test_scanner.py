import serial
import time

# Configura el puerto serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# Comienza la comunicación
ser.write(b'ST\n')
time.sleep(0.1)
# Cambia al modo SCIP 2.0
ser.write(b'SCIP2.0\n')
time.sleep(0.1)
# Establece el rango de medición
ser.write(b'MD0044072500001\n')
time.sleep(0.1)
# Obtiene datos de escaneo
ser.write(b'GD\n')
time.sleep(0.1)
data = ser.readline().decode().strip()
# Procesa los datos
distances = []
angles = []
for i in range(0, len(data), 3):
    distance = ord(data[i]) + (ord(data[i+1]) << 8)
    angle = (i // 3 - 384) * 0.352
    distances.append(distance)
    angles.append(angle)
# Cierra la comunicación
ser.write(b'QT\n')
ser.close()
# Imprime los datos
print(distances)
print(angles)