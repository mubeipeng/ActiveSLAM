import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist, PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
from pylab import *

def plotFeatureMap(x,y,edge,path):
	axis([-15, 5, -5, 15])
	axis('off')
	plt.plot(x[1:],y[1:],'kp',markersize=10)
	for i in range(len(edge)/2):
		e = np.array([edge[2*i].x, edge[2*i+1].x, edge[2*i].y, edge[2*i+1].y])
		plt.plot(e[0:2],e[2:4],'k',linewidth=3.0)
	plt.plot(path[0,0],path[0,1],'or',markersize=10)
	plt.plot(path[:,0],path[:,1],'r',linewidth=2.0)
		#plt.quiver(pose[:,0],pose[:,1],np.cos(pose[:,2]),np.sin(pose[:,2]))
	plt.show()

def multiarray2DToNumpyArray(ma):
    I = ma.layout.dim[0].size
    J = ma.layout.dim[1].size
    na = np.empty([I, J])
    for i in range(I):
        for j in range(J):
            index = ma.layout.data_offset + ma.layout.dim[1].stride * i + j
            na[i, j] = ma.data[index]
    return na

bag = rosbag.Bag('acl_94.bag')
bridge = CvBridge()
pose = np.array([0,0,0])
for topic, msg, t in bag.read_messages(topics=['/TB01/path','/TB01/pose','/TB01/topology_feature_map','/camera/rgb/image_raw']):
	if topic=='/TB01/topology_feature_map':
		x=np.array([0])
		y=np.array([0])
		for i in range(len(msg.markers[0].points)):
			x=np.append(x, np.array([msg.markers[0].points[i].x]))
			y=np.append(y, np.array([msg.markers[0].points[i].y]))		
		edge = msg.markers[1].points
	if topic=='/TB01/path':
		path = multiarray2DToNumpyArray(msg)
		path = np.vstack((current_pose,path))
		print path
		plotFeatureMap(x,y,edge,path)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(0)
	if topic=='/TB01/pose':
		current_pose = np.array([msg.pose.position.x, msg.pose.position.y, 0, 0.2])
		#print p
	#	pose = np.vstack((pose, p))
	if topic=='/camera/rgb/image_raw':
		cv_image = bridge.imgmsg_to_cv2(msg,msg.encoding)
bag.close()


