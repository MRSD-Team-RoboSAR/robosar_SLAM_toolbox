import rospy
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid

pub = rospy.Publisher('/map_filt', OccupancyGrid, queue_size=10)

def filter_map(map_msg):
  scale = map_msg.info.resolution
  origin = [map_msg.info.origin.position.x,
            map_msg.info.origin.position.y]
  # Extract raw map
  raw = np.reshape(
      map_msg.data, (map_msg.info.height, map_msg.info.width))
  # Treat unknown space as occupied space; make binary
  map_filt = np.copy(raw)
  map_filt = np.where(np.logical_or(map_filt<0, map_filt>50), 1, 0)
  # Perform opening to remove small islands of occupied space
  map_filt = ndimage.binary_opening(map_filt, structure=np.ones((3,3))).astype(int) * 100
  # Only update occupied cells
  map_final = np.where(raw > 50, map_filt, raw)
  # Publish new map
  map_msg.data = tuple(map_final.flatten())
  pub.publish(map_msg)



if __name__ == '__main__':
  rospy.init_node('map_filter', anonymous=False)
  rospy.Subscriber("/map", OccupancyGrid, filter_map)
  rospy.spin()

