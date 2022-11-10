import rospy
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid

def filter_map(map_msg):
  scale = map_msg.info.resolution
  origin = [map_msg.info.origin.position.x,
            map_msg.info.origin.position.y]
  # Extract raw map
  raw = np.reshape(
      map_msg.data, (map_msg.info.height, map_msg.info.width))
  plt.imshow(raw, cmap='gray_r')
  plt.show()
  
  # Treat unknown space as occupied space; make binary
  map_filt = np.copy(raw)
  map_filt = np.where(np.logical_or(map_filt<0, map_filt>50), 1, 0)
  plt.imshow(map_filt, cmap='gray_r')
  plt.show()
  # Perform opening to remove small islands of occupied space
  map_filt = ndimage.binary_opening(map_filt, structure=np.ones((3,3))).astype(int)
  plt.imshow(map_filt, cmap='gray_r')
  plt.show()
  # Compare
  raw_bin = np.copy(raw)
  raw_bin = np.where(np.logical_or(raw_bin<0, raw_bin>50), 1, 0)
  diff = np.logical_xor(raw_bin, map_filt)
  plt.imshow(diff, cmap='gray_r')
  plt.show()
  # Only update occupied cells
  map_final = np.where(raw > 50, map_filt, raw)
  plt.imshow(map_final, cmap='gray_r')
  plt.show()


if __name__ == '__main__':
  rospy.init_node('map_filter', anonymous=False)
  rospy.Subscriber("/map", OccupancyGrid, filter_map)
  rospy.spin()

