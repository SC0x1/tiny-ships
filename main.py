#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from framework import *
from boids_system import *
import math
import time


class BaseDynamicEntity(object):
    def __init__(self, name, world, vertices, density, position):
        self._name = name
        self.linear_speed_sqr = 0

        self.body = world.CreateDynamicBody(position=position)
        fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density)
        fixture.filterData.groupIndex = -8

    @property
    def name(self):
        return self._name

    @property
    def linear_velocity(self):
        return self.body.linearVelocity

    @property
    def world_centre(self):
        return self.body.worldCenter

    def world_vector(self, vec=(0,1)):
        return self.body.GetWorldVector(vec)

    @property
    def position(self):
        return self.body.position

    @position.setter
    def position(self, value):
        self.body.position = value

    @property
    def angle(self):
        return self.body.angle

    @angle.setter
    def angle(self, value):
        self.body.angle = value

    @property
    def transform(self):
        return self.body.transform


class ScoutPlane(BaseDynamicEntity):
    vertices = [
        (2.0,  -4.0),
        (0.0,   4.0),
        (-2.0, -4.0), ]

    LINEAR_SPEED = 150
    ANGULAR_SPEED = 1.5
    #ANGULAR_MAX_IMPULSE = 1.5
    ANGULAR_MAX_IMPULSE = 0.1
    MAX_IMPULSE = 3

    MAX_FUEL = 30

    NUM_ACTIVE_PLANES = 0

    def __init__(self, name, world, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None:
            vertices = ScoutPlane.vertices

        BaseDynamicEntity.__init__(self, name, world, vertices, density, position)
        self.body.angularDamping = 3.1
        self.body.linearDamping = 1.1

        self._state_takeoff = False
        self._state_landing = False
        self._try_landing = False
        self._time_start = 0
        self._acceleration = b2Vec2(0, 0)

    @property
    def takeoff(self):
        return self._state_takeoff

    def flight(self):
        if not self._state_takeoff:
            self._time_start = time.time()
            time.clock()
            self._state_takeoff = True
            self.NUM_ACTIVE_PLANES += 1

    def landing(self):
        self._state_takeoff = False
        self._state_landing = False
        self._time_start = 0
        self._acceleration = b2Vec2(0, 0)
        self.NUM_ACTIVE_PLANES -= 1

    def update_linear(self, acceleration):
        self.body.ApplyLinearImpulse(acceleration, self.position, True)
        self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

    def update_angular(self):
        angular_force = self.ANGULAR_SPEED * self.linear_speed_sqr
        if angular_force > self.ANGULAR_MAX_IMPULSE:
            angular_force = self.ANGULAR_MAX_IMPULSE
        self.body.ApplyAngularImpulse(angular_force, True)

    def get_fuel(self):
        fuel = self.MAX_FUEL - (time.time() - self._time_start)
        time.clock()
        if fuel < 0:
            fuel = 0
        return fuel

    def update(self, boids_system, flagship):
        acceleration = b2Vec2(0, 0)
        if self.takeoff:
            if self.get_fuel() > 29:
                acceleration = self.body.GetWorldVector((0, 1))
                acceleration *= 3
            # elif self.get_fuel() == 0:
            #    pass  # plane crashed ?
            elif self.get_fuel() < 15:
                runway_length = 75
                shipdirection = flagship.world_vector((0, 1))
                shipdirection *= -1.0
                shipdirection *= runway_length
                runway_start = shipdirection + flagship.position

                if self._state_landing:
                    # Reynolds’s algorithm (corridor for landing)
                    predict = self.linear_velocity.copy()
                    predict.Normalize()
                    # future location
                    predict *= 10
                    predict_loc = self.position - predict

                    # diff from the runway’s starting location to the plane’s predicted location
                    predict_diff = predict_loc - runway_start

                    # diff from the start of the runway to the end
                    runway_diff = flagship.world_centre - runway_start
                    runway_diff.Normalize()
                    # just scale runway_diff's length
                    runway_diff *= b2Dot(runway_diff, predict_diff)

                    normal_loc = flagship.world_centre + runway_diff

                    distance_outside = b2DistanceSquared(predict_loc, normal_loc)

                    radius = 5  # landing's radius along the runway
                    # if the plane is outside the runway radius seek the target
                    if distance_outside > radius:
                        #target = normal_loc + runway_diff
                        acceleration = boids_system.seek(self, normal_loc)
                    else:
                        acceleration = boids_system.seek(self, flagship.world_centre)

                    distance = b2DistanceSquared(flagship.world_centre, self.world_centre)

                    acceleration *= 0.5

                    if distance < 3000:
                        self._try_landing = True

                    if distance < 5:
                        self.landing()
                    elif (distance > 3000) and self._try_landing:
                        # something went wrong, so we'll just trying again
                        self._state_landing = False
                else:
                    acceleration = boids_system.seek(self, runway_start)
                    acceleration += boids_system.separate(self, 100)
                    if b2DistanceSquared(runway_start, self.position) < 5:
                        self._try_landing = False
                        self._state_landing = True
                        acceleration *= -1.0
            else:
                if self.NUM_ACTIVE_PLANES == 1:
                    acceleration = self.body.GetWorldVector((0, 1))
                    acceleration *= 3
                    acceleration += boids_system.flock(self)
                else:
                    acceleration = boids_system.flock(self)
        else:
            self.position = flagship.world_centre
            self.angle = flagship.angle

        self.update_linear(acceleration)
        self.update_angular()


class FlagShip(BaseDynamicEntity):
    vertices = [(1.5, 0.0),
                (3.0, 5.0),
                (2.8, 11.0),
                (1.0, 20.0),
                (-1.0, 20.0),
                (-2.8, 11.0),
                (-3.0, 5.0),
                (-1.5, 0.0), ]

    LINEAR_SPEED = 50
    ANGULAR_SPEED = 0.1
    ANGULAR_MAX_IMPULSE = 1.5

    MAX_PLANE = 5

    def __init__(self, name, world, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None:
            vertices = FlagShip.vertices
        BaseDynamicEntity.__init__(self, name, world, vertices, density, position)
        self._planes = {}
        self.body.angularDamping = 1.1
        self.body.linearDamping = 1.1

    def update_linear(self, throttle):
        direction = self.body.GetWorldVector((0, 1))
        self.body.ApplyForceToCenter(self.LINEAR_SPEED * throttle * direction, True)
        self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

    def update_angular(self, turn):
        angular_impulse = self.ANGULAR_SPEED * self.linear_speed_sqr
        if angular_impulse > self.ANGULAR_MAX_IMPULSE:
            angular_impulse = self.ANGULAR_MAX_IMPULSE
        self.body.ApplyAngularImpulse(angular_impulse * turn, True)

    def update(self, keys):
        throttle = 0
        if 'up' in keys:
            throttle += 1
        if 'down' in keys:
            throttle -= 1
        self.update_linear(throttle)

        turn = 0
        if 'left' in keys:
            turn += 1
        if 'right' in keys:
            turn -= 1
        self.update_angular(turn)


class ShipGame(Framework):
    name = "Ship Game"
    description = "Keys: accel = w, reverse = s, left = a, right = d, flight = h"

    def __init__(self):
        super(ShipGame, self).__init__()

        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)
        self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'flight'}

        # Keep track of the pressed keys
        self.pressed_keys = set()
        self._is_key_up = False
        self._boidssystem = BoidsSystem()

        # The walls
        boundary = self.world.CreateStaticBody(position=(0, 20))
        walls_vertices = [(-220,-220),
                          (-220, 220),
                          ( 220, 220),
                          ( 220,-220),
                          (-220,-220)]
        boundary.CreateEdgeChain(walls_vertices)

        # A couple regions of differing traction
        gnd1 = self.world.CreateStaticBody()
        fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))

        gnd2 = self.world.CreateStaticBody()
        fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))

        gnd3 = self.world.CreateStaticBody(position=(100, 150))
        #circle=b2CircleShape(pos=(50, 30), radius=1)
        fixture = gnd3.CreateCircleFixture(radius=10)

        self._boidssystem.add_obstacle(ObstaclePoint(gnd1.position))
        self._boidssystem.add_obstacle(ObstaclePoint(gnd2.position))
        self._boidssystem.add_obstacle(ObstacleCircle(gnd3.position, 10))

        self._boidssystem.add_obstacle(ObstacleLine(b2Vec2(-220,-220), b2Vec2(-220, 220)))
        self._boidssystem.add_obstacle(ObstacleLine(b2Vec2(-220, 220), b2Vec2( 220, 220)))
        self._boidssystem.add_obstacle(ObstacleLine(b2Vec2( 220, 220), b2Vec2( 220,-220)))
        self._boidssystem.add_obstacle(ObstacleLine(b2Vec2(-220, 220), b2Vec2(-220,-220)))

        self._planes = [ScoutPlane('scout_' + str(i), self.world) for i in range(5)]
        for plane in self._planes:
            self._boidssystem.add_boid(plane)
        self._num_active_panes = 0

        self._flag_ship = FlagShip('flag_ship', self.world)

        self._boidssystem.add_boid(self._flag_ship)

    def Keyboard(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.add(key_map[key])
        else:
            super(ShipGame, self).Keyboard(key)

    def KeyboardUp(self, key):
        self._is_key_up = True
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.remove(key_map[key])
        else:
            super(ShipGame, self).KeyboardUp(key)

    def Step(self, settings):

        self._flag_ship.update(self.pressed_keys)

        if 'flight' in self.pressed_keys:
            if self._is_key_up:
                for plane in self._planes:
                    if not plane.takeoff:
                        plane.flight()
                        self._is_key_up = False
                        break

        for plane in self._planes:
            plane.update(self._boidssystem, self._flag_ship)

        super(ShipGame, self).Step(settings)
        self.Print('Linear speed sqr: %s' % self._flag_ship.linear_speed_sqr)
        time.sleep(0.001)

if __name__ == "__main__":
     main(ShipGame)

