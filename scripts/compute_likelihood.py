#!/usr/bin/env python

import rospy
import tf
import numpy as np
import map_utils
from nav_msgs.msg import OccupancyGrid, Odometry

class likelihood_field(object):
    
    def __init__(self):
        """ Set up the node, publishers and subscribers. """
        rospy.init_node('mc_localizer')

        # Get values from ROS parameters:
 
        self.laser_sigma_hit = rospy.get_param('~laser_sigma_hit', .2)
        self.map = None
        self.particles = None
	rospy.Subscriber('map', OccupancyGrid, self.map_callback)
	self.likelihood_pub = rospy.Publisher('likelihood_field',
                                              OccupancyGrid, latch=True)

        # Wait for the map
        while not rospy.is_shutdown() and self.map is None:
            rospy.loginfo("Waiting for map.")
            rospy.sleep(.5)
        rospy.spin()

    def map_callback(self, map_msg):
        
        laser_sigma = self.laser_sigma_hit
        self.map = map_msg
        self.grid = np.zeros((map_msg.info.height, map_msg.info.width))
        rospy.loginfo('building Likelihood map... ')
        rospy.logwarn('using laser_sigma = %f',laser_sigma)
        world_map = map_utils.Map(map_msg)

        rospy.loginfo('building KDTree')
        from sklearn.neighbors import KDTree
        occupied_points = []
        all_positions = []
        for i in range(world_map.grid.shape[0]):
            for j in range(world_map.grid.shape[1]):
                all_positions.append(world_map.cell_position(i, j))
                if world_map.grid[i, j] > .9:
                   	occupied_points.append(world_map.cell_position(i, j))

        kdt = KDTree(occupied_points)

        rospy.loginfo('Constructing likelihood field from KDTree.')
        likelihood_field = map_utils.Map(world_map.to_message())
        dists = kdt.query(all_positions, k=1)[0][:]
        probs = np.exp(-(dists**2) / (2 * laser_sigma**2))

        likelihood_field.grid = probs.reshape(likelihood_field.grid.shape)

        rospy.logwarn('Done building likelihood field')
        self.likelihood_pub.publish(likelihood_field.to_message())
         
            
        
if __name__ == "__main__":
    likelihood_field()
