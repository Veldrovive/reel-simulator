import numpy as np
import pymunk
import values
import pygame

class Reel:
    def __init__(self, max_torque, max_rpm, tether_length, inner_radius, tether_diameter, packing_factor):
        self.max_torque = max_torque
        self.max_rpm = max_rpm
        self.max_hz = max_rpm / 60
        self.tether_length = tether_length
        self.inner_radius = inner_radius
        self.tether_diameter = tether_diameter
        self.packing_factor = packing_factor
        self.distance_cutoff = 50

        self.pos = (None, None)
        self.space = None
        self.screen = None
        self.tether_offset = (0, 0)
        self.spool = None
        self.spool_shape = None

        self.curr_used_length = 0
        self.last_used_length = 0

    def create_body(self, space: pymunk.Space, x, y, screen=None, tether_offset=(0, 0)):
        self.space = space
        self.screen = screen
        self.tether_offset = tether_offset
        self.pos = (x, y)
        self.spool = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.spool.position = x, y
        self.spool_shape = pymunk.Circle(self.spool, self.get_cur_radius())
        self.space.add(self.spool, self.spool_shape)

    def get_cur_radius(self):
        return np.sqrt(self.inner_radius**2 + (self.tether_length - self.curr_used_length)/(self.packing_factor * np.pi))

    def get_cur_circumference(self):
        return 2*np.pi*self.get_cur_radius()

    def get_unloaded_force(self):
        return self.max_torque / self.get_cur_radius()

    def get_unloaded_speed(self):
        return self.get_cur_circumference() * self.max_hz

    def get_velocity_factor(self, object_vel):
        max_vel = self.get_unloaded_speed()
        identity_cutoff = max_vel*3/4
        if object_vel < identity_cutoff:
            return 1
        return min(max(1-(object_vel-identity_cutoff)*(1/(max_vel-identity_cutoff)), 0), 1)

    def get_distance_factor(self):
        self.distance_cutoff = 4*self.get_cur_radius()
        if self.curr_used_length > self.distance_cutoff:
            return 1
        return min(max(1-(self.distance_cutoff-self.curr_used_length+5)*(1/self.distance_cutoff), 0), 1)

    def set_rotation_offset(self):
        diff = self.last_used_length - self.curr_used_length
        circumference = self.get_cur_circumference()
        angle_delta = 2*np.pi*diff / circumference
        self.spool.angle -= angle_delta

    def update_view(self):
        self.set_rotation_offset()
        self.spool_shape.unsafe_set_radius(self.get_cur_radius())

    def update_distance(self, object: pymunk.Body):
        to_x, to_y = self.pos
        assert (isinstance(to_x, float) or isinstance(to_x, int)) and (isinstance(to_y, float) or isinstance(to_y, int))
        at_x, at_y = object.position
        if self.screen is not None:
            tether_pos = object.local_to_world(self.tether_offset)
            pygame.draw.lines(self.screen, (153, 43, 43), False, [self.pos, tether_pos])
        direction_vec = np.array([to_x - at_x, to_y - at_y])
        self.last_used_length = self.curr_used_length
        self.curr_used_length = np.linalg.norm(direction_vec)
        return direction_vec

    def get_force(self, object: pymunk.Body, factor: float = 1):
        direction_vec = self.update_distance(object)

        self.update_view()

        assert self.curr_used_length < self.tether_length
        direction_vec = direction_vec / self.curr_used_length

        vel = np.array(object.velocity)
        vel_mag = np.dot(vel, direction_vec)

        # print(f"Reel distance: {self.curr_used_length} - Unloaded force: {self.get_unloaded_force()} - Unloaded speed: {self.get_unloaded_speed()}")

        Fv = self.get_velocity_factor(vel_mag)
        Fd = self.get_distance_factor()

        force_vec = tuple(direction_vec * factor * object.mass * Fv * Fd)
        return force_vec

class ReactiveReel(Reel):
    def __init__(self, space: pymunk.Space, x, y, tethered_object: pymunk.Body, screen=None, tether_offset=(0, 0), distance_cutoff=1000):
        super().__init__(values.torque, values.max_rpm, values.tether_length, values.shaft_diameter/2 + values.inner_radius_padding, values.tether_diameter, values.packing_factor)
        self.space = space
        self.pos = (x, y)
        self.object = tethered_object
        self.distance_cutoff = distance_cutoff

        self.create_body(self.space, *self.pos, screen=screen, tether_offset=tether_offset)

        self.update_distance(self.object)
        self.engaged = self.curr_used_length > self.distance_cutoff

        self.constraint = pymunk.SlideJoint(self.spool, self.object, (0, 0), (0, 0), 0, self.tether_length)

    def step(self):
        self.update_distance(self.object)
        self.update_view()
        if not self.engaged and self.curr_used_length > self.distance_cutoff:
            self.engaged = True
        if self.engaged:
            force = self.get_force(self.object, 2000)
            application_point = self.object.local_to_world((-5, 0))
            self.object.apply_force_at_world_point(force, application_point)
