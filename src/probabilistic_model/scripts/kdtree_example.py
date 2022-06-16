import sys
import time
import numpy as np
import cv2
from scipy import spatial


if __name__ == '__main__':

  if len(sys.argv) != 3:
    print( 'Usage: %s row column' % (sys.argv[0]) )
    sys.exit(0)

  pix_row = int( sys.argv[1] )
  pix_col = int( sys.argv[2] )

  print( 'pixel: (%d, %d)' % (pix_row, pix_col) )

  # load map
  mapimg = cv2.imread( 'global_map.png', cv2.IMREAD_GRAYSCALE )

  # Linear scan
  min_dist = float('inf')
  closest_obstacle = None
  start = time.clock()
  for h in range( mapimg.shape[0] ):
    for w in range( mapimg.shape[1] ):
      if mapimg[h, w] == 0:
        dist = np.hypot( pix_row - h, pix_col - w )
        if min_dist > dist:
          min_dist = dist
          closest_obstacle = [h, w]
  end = time.clock()
  print( '[LINEAR SCAN] closest obstacle: %s - dist: %f - time: %s [s]' % (str(closest_obstacle), min_dist, str(end - start)) )

  # KDTree
  obstacle_coords = list()
  for h in range( mapimg.shape[0] ):
    for w in range( mapimg.shape[1] ):
      if mapimg[h, w] == 0:
        obstacle_coords.append( [h, w] )
  tree = spatial.KDTree( obstacle_coords )  

  start = time.clock()
  dist, point_id = tree.query( [[pix_row, pix_col]] )
  end = time.clock()
  print( '[KDTree     ] closest obstacle: %s - dist: %f - time: %s [s]' % (obstacle_coords[point_id[0]], dist[0], str(end - start)) )


