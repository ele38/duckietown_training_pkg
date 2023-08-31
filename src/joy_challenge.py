#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped


class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy
        self.sub = rospy.Subscriber("/duckiebot/joy" , Joy, self.callback)
        #publicar la intrucciones del control en possible_cmd
        self.publi = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 10)
        self.twist = Twist2DStamped()


    def publicar(self, msg):
        self.publi.publish(msg)

    def callback(self,msg):

        #a = msg.buttons[]
        #y = msg.axes[1]
        x = msg.axes[0]
        z = (1 - msg.axes[5])/2 #boton RT
        z2 = (1 - msg.axes[2])/2 #boton LT

        print(y, x, z)
        self.twist.omega = -x * 10
        self.twist.v = z - z2


        self.publi.publish(self.twist)


def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #obj.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
