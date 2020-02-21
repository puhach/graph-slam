from math import *
import random


### ------------------------------------- ###
# Below, is the robot class
#
# This robot lives in 2D, x-y space, and its motion is
# pointed in a random direction, initially.
# It moves in a straight line until it comes close to a wall 
# at which point it stops.
#
# For measurements, it  senses the x- and y-distance
# to landmarks. This is different from range and bearing as
# commonly studied in the literature, but this makes it much
# easier to implement the essentials of SLAM without
# cluttered math.
#
class robot:
    
    def __init__(self, world_size = 100.0, measurement_range = 30.0,
                 motion_noise = 1.0, measurement_noise = 1.0):	
        """
        Creates a robot with the specified parameters and initializes 
        the location (self.x, self.y) to the center of the world.
        """
        self.measurement_noise = 0.0
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = world_size / 2.0
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = []
        self.num_landmarks = 0
        
    
    def rand(self):
        """
        Returns a positive, random float.
        """
        return random.random() * 2.0 - 1.0
    

    def move(self, dx, dy):
        """
        Attempts to move robot by dx, dy. If outside world boundary, 
        then the move does nothing and instead returns failure.
        """
        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise

        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            self.x = x
            self.y = y
            return True


    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 to make all
    #        landmarks visible at all times
    #
    
    def sense(self):
        """ 
        This function does not take in any parameters, instead it references internal variables
        (such as self.landamrks) to measure the distance between the robot and any landmarks
        that the robot can see (that are within its measurement range). The function accounts 
        for measurement_noise and measurement_range.

        Returns a list of landmark indices, and the measured distances (dx, dy) between the robot's 
        position and said landmarks. One item in the list will be in the form: [landmark_index, dx, dy].
        """
            
        measurements = []

        ## For each landmark
        ## 1. compute dx and dy, the distances between the robot and the landmark
        ## 2. account for measurement noise by *adding* a noise component to dx and dy
        ##    - The noise component should be a random value between [-1.0, 1.0)*measurement_noise
        ##    - Feel free to use the function self.rand() to help calculate this noise component
        ## 3. If either of the distances, dx or dy, fall outside of the internal var, measurement_range
        ##    then we cannot record them; if they do fall in the range, then add them to the measurements list
        ##    as list.append([index, dx, dy]), this format is important for data creation done later

        for i, lk in enumerate(self.landmarks):
            dx = lk[0] - self.x
            dy = lk[1] - self.y
            dx += self.rand()*self.measurement_noise
            dy += self.rand()*self.measurement_noise
            d = sqrt(dx*dx + dy*dy)
            
            if d <= self.measurement_range:
                measurements += [[i, dx, dy]]
                #measurements += [(i, dx, dy)]

        return measurements


    def make_landmarks(self, num_landmarks):
        """
        Make random landmarks located in the world.
        """
        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                    round(random.random() * self.world_size)])
        self.num_landmarks = num_landmarks


    
    def __repr__(self):
        """
        Called when print(robot) is called; prints the robot's location.
        """
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)



####### END robot class #######