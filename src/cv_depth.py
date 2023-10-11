#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32, Float32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy




class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		#Suscribrirse a la camara
		self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.procesar_img)
        #Publicar imagen(es)
		self.pub_img = rospy.Publisher("/duckiebot/distancia", Float32, queue_size = 1)
		#self.pub_img = rospy.Publisher("mas", Image, queue_size = 1)


	def procesar_img(self, msg):
		#Transformar Mensaje a Imagen
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")

		#Espacio de color

			#cv2.COLOR_RGB2HSV
			#cv2.COLOR_RGB2GRAY
			#cv2.COLOR_RGB2BGR
			#cv2.COLOR_BGR2HSV
			#cv2.COLOR_BGR2GRAY
			#cv2.COLOR_BGR2RGB

		image_out = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

		#Definir rangos para la mascara

		lower_limit_pato = np.array([20,100,100])
		upper_limit_pato =np.array([40,255,255])
		lower_limit_cosa = np.array([170,20,20])
		upper_limit_cosa = np.array([210,100,100])
		#Mascara
		mask_pato = cv2.inRange(image_out, lower_limit_pato, upper_limit_pato)
		image_out_pato = cv2.bitwise_and(image, image, mask=mask_pato)
		mask_cosa = cv2.inRange(image_out, lower_limit_cosa, upper_limit_cosa)
		image_out_cosa = cv2.bitwise_and(image,image,mask=mask_cosa)

		# Operaciones morfologicas, normalmente se utiliza para "limpiar" la mascara
		kernel = np.ones((2 , 2), np.uint8)
		img_erode_pato = cv2.erode(mask_pato, kernel, iterations=2) #Erosion
		img_dilate_pato = cv2.dilate(img_erode_pato, kernel, iterations=2) #Dilatar 
		img_erode_cosa = cv2.erode(mask_cosa, kernel, iterations=2) #Erosion
		img_dilate_cosa = cv2.dilate(img_erode_cosa, kernel, iterations=2) #Dilatar 

		# Definir blobs
		_,contours_pato, hierarchy_pato = cv2.findContours(img_dilate_pato, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		_,contours_cosa, hierarchy_cosa = cv2.findContours(img_dilate_cosa,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		msg = "pato: "
		dist_min = 10000
		for cnt in contours_pato:
			AREA = cv2.contourArea(cnt)
			if AREA>10: #Filtrar por tamano de blobs
				x,y,w,h = cv2.boundingRect(cnt)
				p = h #tamano en pixeles
				dr = 25 #tamano real
				f = 320 # distancia focal
				Dr = dr*f/p #distancia
				dist_min = min(dist_min, Dr)
		self.pub_img.publish(dist_min)
#		msg += "\n cosa: "
		for cnt in contours_cosa:
			AREA = cv2.contourArea(cnt)
			if AREA>10: #Filtrar por tamano de blobs
				x,y,w,h = cv2.boundingRect(cnt)
				p = h #tamano en pixeles
				dr = 25 #tamano real
				f = 320 # distancia focal
				Dr = dr*f/p #distancia
#				msg += str(Dr)
				
				#self.pub_img.publish(msg) 
			else:
				None

#		self.pub_img.publish(msg) 
		# Publicar imagen final
#		msg = bridge.cv2_to_imgmsg(image, "bgr8")
#		self.pub_img.publish(msg)

def main():
	rospy.init_node('detection') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
