#!/usr/bin/env python3

import io
import pynmea2
import serial
import re

import rospy
#from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion



ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=2.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

def dm_to_sd(dm):
    '''
    Converts a geographic co-ordinate given in 
    "degrees/minutes" dddmm.mmmm
    format (eg, "12319.943281" = 123 degrees,
    19.943281 minutes)
    to a signed decimal (python float) format
    '''
    # '12319.943281'
    if not dm or dm == '0':
        return 0.
    d, m = re.match(r'^(\d+)(\d\d\.\d+)$', dm).groups()
    return float(d) + float(m) / 60

def node():

    rospy.init_node('gps_publish')
    pub = rospy.Publisher('gps_node',Quaternion, queue_size=10)

    while 1:
        try:
            msg1 = Quaternion()
            line = sio.readline()
            if(line.startswith("$GPGGA")):
                msg = pynmea2.parse(line)
                latitude = msg.lat
                longitude = msg.lon

                latitude_parse = dm_to_sd(latitude)
                longitude_parse = dm_to_sd(longitude)

                msg1.x = float(latitude_parse)
                msg1.y = float(longitude_parse)

                if msg.lon_dir == 'W':
                    #longitud_parse_1
                    longitud = msg.lon
                    longitud_parse = dm_to_sd(longitud)
                    longitud_parse_1 = longitud_parse * -1
                    
                elif msg.lon_dir == 'E':
                    longitud = msg.lon
                    longitud_parse = dm_to_sd(longitud)
                    longitud_parse_1 = longitud_parse * 1
                
                if msg.lat_dir == 'N':
                    #latitud_parse_1
                    latitud = msg.lat
                    latitud_parse = dm_to_sd(latitud)
                    latitud_parse_1 = latitud_parse * 1
                elif msg.lat_dir == 'S':
                    #global latitud_parse_2
                    latitud = msg.lat
                    latitud_parse = dm_to_sd(latitud)
                    latitud_parse_1 = latitud_parse * -1


                msg1.x = float(longitud_parse_1)
                msg1.y = float(latitud_parse_1)

               
            elif(line.startswith("$GPVTG")):
                msg = pynmea2.parse(line)  
                vel = msg.spd_over_grnd_kmph
                if(vel != ''):
                    msg1.z = float(vel)
                else:
                    rospy.logwarn('No hay valores dentro del area')
                #azimut = msg.azimut_1
       
            elif(line.startswith("$GPGSV")):

                msg = pynmea2.parse(line)
                azimuth1 = msg.azimuth_1
                if(azimuth1 != ''):
                    msg1.w = float(azimuth1)
                else:
                    rospy.logwarn('No hay valores dentro del area')
               
            pub.publish(msg1)
            rate = rospy.Rate(10)
            rate.sleep()

        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            break
        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            continue
        except Exception:
            return None
            #return self.sp.readline().decode('UTF-8').replace("\r\n", "")

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            node()
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass