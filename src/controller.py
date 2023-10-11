#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32, Float32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped


class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy
        self.sub = rospy.Subscriber("/duckiebot/possible_cmd/duckietown_msg" , Twist2DStamped, self.callback_joy)
        self.sub2 = rospy.Subscriber("/duckiebot/distancia", Float32, self.callback_dist)
#publicar la intrucciones del control en possible_cmd
        self.publi = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 10)
	self.cerca = False
    def callback_dist(self, dist):
	if dist < 250:
		print("cualquier wea -gonzalo")
		self.cerca = True
	else:
		self.cerca = False
    def callback_joy(self, joy):
	temp = joy
	if self.cerca:
		temp.v = 0
	self.publi.publish(temp)

    def publicar(self, msg):
        self.publi.publish(msg)

def main():
    rospy.init_node('controller') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #obj.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
