import numpy as np
from numpy.linalg import norm

dt = 0.1

class Drone():

    def __init__(self, pos):

        self.pos = pos
        self.vel = np.zeros(2)
        self.error_prev = np.zeros(2)
        self.u_max = 2
        self.v_max = 2

    def move(self, target):

        u = self.control(target)
        self.vel += u * dt

        if norm(self.vel) > self.v_max:
            self.vel = self.vel / norm(self.v) * self.v_max

        self.pos += self.vel * dt

    def control(self, target):
        k_p = 10; k_d = 10
        error = target - self.pos
        d_error = (error - self.error_prev) / dt

        self.error_prev = error
        u = k_p * error + k_d * d_error
        if norm(u) > self.u_max:
            u = (u / norm(u)) * self.u_max

        return u


def main():

    init_pos = np.array([1.0, 1.0])
    drone = Drone(init_pos)





if __name__ == '__main__':
    main()
