import numpy as np

import rospy


class ImpactLocationOnFarWall():
    def __init__():
        self.w = 0.6858
		self.l = 1.3843
		self.w_goal = 0.29845
		self.gripper_offset = 0.04
		self.our_side = 'left'


	def impact_location_on_far_wall(self, x, y, tan_theta):
        unwalled_impact_location = x + y * tan_theta
		
		impact_location_sign = np.sign(unwalled_impact_location)
		
		limited_impact_location = np.abs(unwalled_impact_location) % 2*self.w

        if limited_impact_location < 0.5*self.w:
            return impact_location_sign * limited_impact_location
	    elif limted_impact_location < 1.5*self.w:
            return impact_location_sign * ( self.w - limited_impact_location)
	    else:
		    return impact_location_sign * ( limited_impact_location - 2.0*self.w)

    def target_position(self, x, y, tan_theta):
        impact_location = self.impact_location_on_far_wall(x, y, tan_theta)
		return np.clip(-0.5*self.w_g + gripper_offset, 
						0.5*self.w_g - gripper_offset, 
						impact_location)
