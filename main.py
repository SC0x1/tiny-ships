#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time

g_is_key_up = False


class BoidsSystem:
    def __init__(self):
        self._boids = {}
        self._obstacles = []

    def add_boid(self, boid):
        if boid:
            self._boids[boid.name] = boid
            return True
        return False

    def add_obstacle(self, obstacle):
        if obstacle:
            self._obstacles.append(obstacle)

    def update(self, keys):
        flagship = self._boids.get("flag_ship", None)
        if flagship:
            flagship.update(keys)

            if 'flight' in keys:
                global g_is_key_up
                if g_is_key_up:
                    for plane in self._boids.itervalues():
                        if isinstance(plane, ScoutPlane):
                            if not plane.takeoff:
                                plane.flight()
                                g_is_key_up = False
                                break

            transform = b2Transform(b2Vec2(flagship.world_centre), b2Rot(flagship.angle))
            for boid in self._boids.itervalues():
                if isinstance(boid, ScoutPlane):
                    boid.update(self._boids, self._obstacles, transform)

'''
    Obstacles
'''
class ObstacleLine:
    def __init__(self, loc1, loc2):
        self._loc1 = loc1.copy()
        self._loc2 = loc2.copy()
        loc1 += loc2
        self._centroid = loc1.copy()
        self._centroid /= 2

    def centroid(self):
        return self._centroid

    def closest_point(self, pos):
        diffx = self._loc2.x - self._loc1.x
        diffy = self._loc2.y - self._loc1.y
        u = ((pos.x - self._loc1.x)*diffx + (pos.y - self._loc1.y)*diffy) / (diffx*diffx + diffy*diffy)
        u = max(min(u, 1), 0)
        return b2Vec2(self._loc1.x + u * diffx, self._loc1.y + u * diffy)


class ObstaclePoint:
    def __init__(self, loc):
        self._location = b2Vec2(loc)

    def centroid(self):
        return self._location

    def closest_point(self, pos):
        return self._location


class ObstacleCircle:
    def __init__(self, loc, radius):
        self._location = b2Vec2(loc)
        self._radius = radius

    def centroid(self):
        return self._location

    def closest_point(self, pos):
        radial = pos - self._location
        radial.Normalize()
        radial *= self._radius
        radial += self._location
        return radial


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

    def __init__(self, name, world, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None:
            vertices = ScoutPlane.vertices

        BaseDynamicEntity.__init__(self, name, world, vertices, density, position)
        self.body.angularDamping = 3.1
        self.body.linearDamping = 1.1

        self._takeoff = False
        self._time_start = 0
        self._acceleration = b2Vec2(0, 0)
        self.base_position = b2Vec2(0, 0)

    @property
    def takeoff(self):
        return self._takeoff

    def flight(self):
        if not self._takeoff:
            self._time_start = time.time()
            time.clock()
            self._takeoff = True

    def landing(self):
        self._takeoff = False
        self._time_start = 0
        self._acceleration = b2Vec2(0, 0)

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

    def flock(self, boids, obstacles):
        separation = self.separate(boids)
        align = self.align(boids)
        cohesion = self.cohesion(boids)
        avd = self.avoid(obstacles, 30)

        avd *= 25
        separation *= 1.5
        align *= 1.0
        cohesion *= 1.1

        return separation + align + cohesion + avd

    def seek(self, target):
        desired = target - self.position
        if desired.length == 0:
            return b2Vec2(0, 0)

        desired.Normalize()
        desired *= self.LINEAR_SPEED

        steer = desired - self.linear_velocity

        if steer.length > self.MAX_IMPULSE:
            steer.Normalize()
            steer *= self.MAX_IMPULSE
        return steer

    def steer_to_base(self, target, obstacles):
        desired = target - self.position
        desired += self.avoid(obstacles, 30)
        desired *= 0.05
        self.body.ApplyLinearImpulse(desired, self.position, True)
        distance = b2DistanceSquared(target, self.position)
        if distance < 5:
            self.landing()

    def steer(self, target):
        force = b2Vec2(0, 0)
        desired = target - self.position
        distance = desired.length

        if distance > 0:
            desired.Normalize()
            if distance < 100.0:
                desired *= 0.05
            else:
                desired *= self.LINEAR_SPEED

            force = desired - self.linear_velocity
            if force.length > self.MAX_IMPULSE:
                force.Normalize()
                force *= self.MAX_IMPULSE
        return force

    def separate(self, neighbours):
        separation = 2500
        steer = b2Vec2(0, 0)
        count = 0
        for boid in neighbours.itervalues():
            distance = b2DistanceSquared(self.position, boid.position)
            '''if isinstance(boid, FlagShip):
                separation = 300'''
            if (distance > 0) and (distance < separation):
                diff = self.position - boid.position
                diff.Normalize()
                diff *= 1.0/distance
                steer += diff
                count += 1

        if count > 0:
            steer *= 1.0/count

        if steer.length > 0:
            steer.Normalize()
            steer *= self.LINEAR_SPEED
            steer -= self.linear_velocity
            if steer.length > self.MAX_IMPULSE:
                steer.Normalize()
                steer *= self.MAX_IMPULSE
        return steer

    def align(self, neighbours):
        neighbordist = 10000
        steer = b2Vec2(0, 0)
        count = 0

        for boid in neighbours.itervalues():
            distance = b2DistanceSquared(self.position, boid.position)
            if (distance > 0) and (distance < neighbordist):
                steer += boid.linear_velocity
                count += 1

        if count > 0:
            steer *= 1.0/count

        if steer.length > 0:
            steer.Normalize()
            steer *= self.LINEAR_SPEED
            steer -= self.linear_velocity
            if steer.length > self.MAX_IMPULSE:
                steer.Normalize()
                steer *= self.MAX_IMPULSE
        return steer

    def cohesion(self, neighbours):
        neighbordist = 100000
        sum = b2Vec2(0, 0)
        count = 0
        for boid in neighbours.itervalues():
            distance = b2DistanceSquared(self.position, boid.position)
            if (distance > 0) and (distance < neighbordist):
                sum += boid.position
                count += 1

        if count > 0:
            sum *= 1.0/count
            return self.seek(sum)
        else:
            return b2Vec2(0, 0)

    def avoid(self, obstacles, radius):
        avd = b2Vec2(0, 0)
        for ob in obstacles:
            distance = b2DistanceSquared(self.position, ob.centroid())
            if distance < 3000:
                closest = ob.closest_point(self.position)
                if b2DistanceSquared(self.position, closest) <= radius:
                    force = self.steer(closest)
                    force *= -1.0
                    force *= self.MAX_IMPULSE
                    avd += force
        return avd

    def update(self, boids, obstacles, transform):
        acceleration = b2Vec2(0, 0)
        if self.takeoff:
            if self.get_fuel() > 29:
                acceleration = self.body.GetWorldVector((0, 1))
                acceleration *= 3
            elif self.get_fuel() < 15:
                self.steer_to_base(transform.position, obstacles)
            else:
                acceleration = self.flock(boids, obstacles)
        else:
            self.position = transform.position
            self.angle = transform.angle

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

        for b in [ScoutPlane('scout_'+str(i), self.world) for i in range(5)]:
            self._boidssystem.add_boid(b)

        self.ship1 = FlagShip('flag_ship', self.world)

        self._boidssystem.add_boid(self.ship1)

    def Keyboard(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.add(key_map[key])
        else:
            super(ShipGame, self).Keyboard(key)

    def KeyboardUp(self, key):
        global g_is_key_up
        g_is_key_up = True
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.remove(key_map[key])
        else:
            super(ShipGame, self).KeyboardUp(key)

    def Step(self, settings):
        self._boidssystem.update(self.pressed_keys)

        super(ShipGame, self).Step(settings)
        self.Print('Linear speed sqr: %s' % self.ship1.linear_speed_sqr)
        time.sleep(0.001)

if __name__ == "__main__":
     main(ShipGame)

