import numpy as np
from numpy.linalg import norm
import conf

dt=conf.dt # assume time step always 0.1

class Repeater():
    """
    Class for the repeater drone
    """
    def __init__(self, pos,index,a_max,v_max):
        """
        The repeater will have a position and a velocity.
        The other variables are needed for the PD controller.
        :param pos: initial position of the drone (usually set to the ground station position)
        :param index: priority index of the repeater (lower index, higher priority in the swarm).
                    The repeaters must avoid collision with other repeaters with higher priority
        :param a_max: maximum acceleration for the repeater
        :param v_max: maximum velocity for the repeater
        """
        self.position = pos
        self.velocity = np.zeros(2)
        self.pos_desired=None

        self.a_max=a_max
        self.v_max=v_max

        #parameters for controller and force field
        # todo: choose good parameters for the gains
        # Proportional and differential gains
        self.kp = conf.kp
        self.kd = conf.kd

        # repulsive repeater gain, repulsive shaded gain, noise gain
        self.G_r = conf.G_r
        self.G_s = conf.G_s
        self.G_n = conf.G_n


        self.error_prev = np.zeros(2)
        self.index=index

        # For noise in the control
        self.percistance_counter = 0
        self.noise_direction = np.zeros(2)

    def move(self, pos_desired, pos_main_drone, repeaters, boundary_centers):

        """
        Updates the position and velocity of the repeater.
        :param pos_desired: Desired position to go towards
        :param pos_main_drone: Position of the main drone. Needed to avoid collision
        :param repeaters: A list with all the repeater drones. To avoid collision
        :param boundary_centers: A list with the center coordinates of the boundary
                between seen and unseen area
        :return:
        """

        # Get the control signal (acceleration)
        u = self.control(pos_desired, pos_main_drone, repeaters, boundary_centers)

        # Make sure it's not too large
        if norm(u) > self.a_max:
            u = u / norm(u) * self.a_max

        # Update velocity
        self.velocity += u * dt

        # Make sure it's not too large
        if norm(self.velocity) > self.v_max:
            self.velocity = self.velocity / norm(self.velocity) * self.v_max

        # Update position
        self.position += self.velocity*dt


    def get_repulsive(self, centers, S = 5, R = 3):
        """

        :param centers: discretized boundaries of the unaccessible areas.
        :param S: distance of force field effect.
        :param R: critic disnace. If repeater get closer than R the repulsion is bigger.
        :return: total repulsive force from all the obstacles and unseen areas
        """

        repulsive = np.zeros(2)

        for center in centers:
            d = norm(center - self.position)

            # If repeater is far away, don't account for it at all
            # or if we have priority
            if d > S:
                continue

            # Direction from repeater to self, normalized
            direction = (center - self.position) / d

            # If very close, large repulsive force
            if d <= R:
                repulsive += direction * 100

            else:
                repulsive += ((S - d) / (S - R)) * direction

        return repulsive



    def get_repulsive_repeaters(self, repeaters, S = 5, R = 3):
        """
        Compute repulsive force from the other drones with higher priority
        :param repeaters: list of repeaters in the mine
        :param S: distance of force field effect.
        :param R: critic disnace. If repeater get closer than R the repulsion is bigger.
        :return: total repulsive force from all the repeaters
        """

        repulsive = np.zeros(2)

        # Compute repulsive forces from repeaters
        if len(repeaters) > 1:  # If more repeaters than this one
            for repeater in repeaters:  # Look at all repeaters except for this one. Assign indices? d == 0?
                d = norm(repeater.position - self.position)

                # If repeater is far away, don't account for it at all
                # or if we have priority
                if d > S or d == 0 or self.index < repeater.index:
                    continue

                # Direction from repeater to self, normalized
                direction = (repeater.position - self.position) / d

                # If very close, large repulsive force
                if d <= R:
                    repulsive += direction * 100

                else:
                    repulsive += ((S - d) / (S - R)) * direction

        return repulsive


    def get_repulsive_main(self, pos_main_drone, S = 5, R = 3):
        """
        Compute repulsive force from the main drone
        :param pos_main_drone: position of the main drone
        :param S: distance of force field effect.
        :param R: critic disnace. If repeater get closer than R the repulsion is bigger.
        :return: repulsive force from the main drone
        """

        repulsive = np.zeros(2)

        d = norm(pos_main_drone - self.position)

        # Direction from main drone to self, normalized
        direction = (pos_main_drone - self.position) / d

        # If very close, large repulsive force
        if d <= R:
            repulsive += direction * 100

        else:
            repulsive += ((S - d) / (S - R)) * direction

        return repulsive

    def get_noise(self):
        """
        Additional noisy repulsion to help get out from local minima
        :return:
        """
        if self.percistance_counter % 10 == 0:
            #The noise is changed every 10 time steps
            noise_direction = np.random.normal(0, 1, 2)
            self.noise_direction = noise_direction / norm(noise_direction)

        self.percistance_counter += 1

        return self.noise_direction


    def control(self, pos_desired, pos_main_drone, repeaters, boundary_centers):

        """
        PD controller to provide a control signal / acceleration
        :param pos_desired: Desired position to go towards
        :param pos_main_drone: Position of the main drone
        :param repeaters: A list with all the repeater drones. To avoid collision
        :param boundary_centers: A list with the center coordinates of the boundary
        between seen and unseen area
        :return: Control signal u
        """

        ### Repeaters
        repulsive_repeaters = self.G_r * self.get_repulsive_repeaters(repeaters)

        #### Boundary of non-seen
        repulsive_shaded = self.G_s * self.get_repulsive(boundary_centers)

        #### Main drone
        repulsive_main_drone = self.G_r * self.get_repulsive_main(pos_main_drone)

        ### Noise
        noise = self.G_n * self.get_noise()


        # Accumulated repulsive force from obstacles, other drones and unseen area.
        repulsive = - repulsive_repeaters - repulsive_shaded - repulsive_main_drone

        # PD controller
        error = pos_desired - self.position
        d_error = (error - self.error_prev) / dt

        # Control signal
        u = self.kp * error + self.kd * d_error + repulsive + noise

        self.error_prev = error

        return u