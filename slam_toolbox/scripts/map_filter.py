import rospy
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid

pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

def filter_map(map_msg):
  scale = map_msg.info.resolution
  origin = [map_msg.info.origin.position.x,
            map_msg.info.origin.position.y]
  # Extract raw map
  raw = np.reshape(
      map_msg.data, (map_msg.info.height, map_msg.info.width))
  # Treat unknown space as free space; make binary
  map_filt = np.copy(raw)
  map_filt = np.where(map_filt<50, 0, 100)
  # Perform Gaussian blur, then threshold
  map_filt = ndimage.gaussian_filter(map_filt, sigma=3)
  map_filt = np.where(map_filt > 6, 100, 0)
  # Only update occupied cells
  map_final = np.where(raw > 50, map_filt, raw)
  # Publish new map
  map_msg.data = tuple(map_final.flatten())
  pub.publish(map_msg)



if __name__ == '__main__':
  rospy.init_node('map_filter', anonymous=False)
  rospy.Subscriber("/slam_toolbox/map", OccupancyGrid, filter_map)
  rospy.spin()

