#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
import std_msgs.msg
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import tf
import time
import RPi.GPIO as GPIO  # Importamos la libreria RPi.GPIO

if __name__ == '__main__':
    pub = rospy.Publisher('joint_states', JointState, queue_size=2)
    rospy.init_node('state_publisher')
    br = tf.TransformBroadcaster()
    joint_state = JointState()
    rate = rospy.Rate(30)
    degree = 0
    angle = degree*(math.pi / 180.0)
    GPIO.setmode(GPIO.BOARD)  # Ponemos la Raspberry en modo BOARD
    GPIO.setup(21, GPIO.OUT)  # Ponemos el pin 21 como salida
    p = GPIO.PWM(21, 50)  # Ponemos el pin 21 en modo PWM y enviamos 50 pulsos por segundo
    p.start(7.5)  # Enviamos un pulso del 7.5% para centrar el servo


    while not rospy.is_shutdown():

        try:
            def y(x):
                return (((x - 225.0) / 90) * 10.0 ** -3) * 50.0 * 100.0

            for i in range(1,92,1):
                degree=i-1
                angle = 45-(degree * (math.pi / 180.0))
                p.ChangeDutyCycle(y(i+74))
                br.sendTransform((0, 0, 0),
                             tf.transformations.quaternion_from_euler(0, -angle, 0),
                             rospy.Time.now(),
                             "lidarbase_link",
                             "basefootprint_link")
                print degree
                time.sleep(0.2)

            p.stop()
            rospy.spin()
            rate.sleep()


     
        except rospy.ROSInterruptException:
            GPIO.cleanup()
            pass


