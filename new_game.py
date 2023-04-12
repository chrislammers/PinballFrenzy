"""
Starting new again...
"""


import pygame
import numpy as np
from scipy.integrate import ode
import sys

# colors:
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GREY = (50, 50, 50)

# Class Structure:
# everything is a Body
#

clock = pygame.time.Clock()

# 1:1 scale
window = [500, 800]
fieldx = [0, window[0]]
fieldy = [0, window[1]]

class Ball(pygame.sprite.Sprite):

    def __init__(self, name, mass, color=RED, radius=1):
        pygame.sprite.Sprite.__init__(self)

        self.image = pygame.Surface([radius*2, radius*2])
        self.image.set_colorkey(BLACK)
        pygame.draw.circle(self.image, color, (radius, radius), radius, 3)
        self.rect = self.image.get_rect()

        self.state = np.zeros(4)
        self.state[:2] = window
        self.mass = mass
        self.radius = radius
        self.name = name
        # self.distances = []
        self.tol_distance = 0.01
        self.grav = [0, 3]

        self.solver = ode(self.f)
        self.solver.set_integrator('dop853')
        self.solver.set_initial_value(self.state)

    def set_pos(self, pos):
        self.state[0:2] = pos
        self.solver.set_initial_value(self.state)

    def get_pos(self):
        return self.state[0:2]

    def set_vel(self, vel):
        self.state[2:4] = vel
        self.solver.set_initial_value(self.state)

    def get_vel(self):
        return self.state[-2:]

    def get_mass(self):
        return self.mass

    def down_under(self):
        # check if the ball is below the game, reset ball
        if self.state[1]>=fieldy[1]:
            self.set_pos([500-self.radius-1,500])
            self.set_vel([0,-100])

    def wall_coll(self, state):
        return state[0]<=fieldx[0]+self.radius or state[0]>=fieldx[1]-self.radius or state[1]<=fieldy[0]+self.radius

    def wall_coll_resp(self, state, t, dt):
        side = True
        if state[1]-self.radius < self.tol_distance:
            # print("ceiling")
            side = False

        beg = t-dt
        end = t
        mid = (beg+end)/2
        # print("between",beg,"and", end)
        # print(state[0],self.tol_distance)
        # print("--while loop--")
        iterations = 0
        # print(state[0]-fieldx[1]+self.radius)
        # print(state[1]-fieldy[0]-self.radius)
        # print(state[0]+fieldx[0]-self.radius)

        while state[0]-fieldx[1]+self.radius > self.tol_distance and state[1]-fieldy[0]-self.radius > self.tol_distance and state[0]+fieldx[0]-self.radius > self.tol_distance:
            # print("between",beg,"and", end)
            # print(state[0]-fieldx[1]+self.radius)

            mid = (beg+end)/2
            state = self.solver.integrate(mid)
            if self.wall_coll(state):
                end = mid
                # t = mid
            else:
                beg = mid

            iterations+=1
            if iterations > 150:
                # accuracy limiter
                break
        # print("collision at", mid)
        # find out if the collision is on the walls or roof.
        if not side:
            # print("Ceiling or floor bounce. New state:")
            # print([state[0], state[1], state[2], -1*state[3]])
            return np.array([state[0], state[1], state[2], -1*state[3]]), mid
        else:
            # print("Wall bounce. New state:")
            # print([state[0], state[1], -1*state[2], state[3]])
            return np.array([state[0], state[1], -1*state[2], state[3]]), mid

    def bumper_coll(self, state, other):
        col = False

        rA = self.radius
        rB = other.radius
        xA = self.state[:2]
        xB = other.state[:2]

        d = (xA-xB)

        r_c = np.linalg.norm(d)
        r = r_c-rA-rB
        u = d/r_c

        if r<0:
            # print("contact")
            vA = self.state[-2:]
            vB = other.state[-2:]
            vAB = vA-vB

            che = np.dot(vAB, u)
            # print(vAB, "dot", n, " = ",che)
            if che > 0:
                pass
                # print("moving away")
            elif che < 0:
                # print("Collision!")
                col = True
            else:
                pass
                # print("resting contact")



        return col
        pass

    def bumper_coll_resp(self, state, other, t, dt):
        rA = self.radius
        rB = other.radius
        xA = self.state[:2]
        xB = other.state[:2]

        d = xA-xB
        r_c = np.linalg.norm(d)
        r = r_c-rA-rB
        n = d/r_c

        vA = self.state[-2:]
        vB = other.state[-2:]
        vAB = vA-vB

        j = (-2*np.dot(vAB, n))/(1/self.mass + 1/other.mass)

        v2A = vA + (j*n)/self.mass
        self.state[-2:] = v2A * other.bounce
        self.solver.set_initial_value(self.state, t+dt)


    def f(self, t, state):
        dx = state[2]
        dy = state[3]
        dvx = self.grav[0]
        dvy = self.grav[1]
        return [dx, dy, dvx, dvy]

    def update1(self, objects, dt):
        # print("updating: ", self.name)

        # objects is a dict of non-ball objects that the ball can collide with.
        #   the walls (basically, the window itself.)
        #   the bumpers (circles, easy collisions)
        #   the two flippers (harder collisions, will think about this)
        #   the initial spring.

        # new_state = self.solver.integrate(self.solver.t+dt)

        for o in objects:
            # print("updating: ", self.name)

            other = objects[o]
            # self.self.solver.set_f_params(other)

            new_state = self.solver.integrate(self.solver.t+dt)

            # check if the ball is out the bottom (game over, reset ball)
            # check if the ball is going to hit a wall (window edge, not bottom)
            # check if ball is going to hit bumper, flipper, and (maybe) spring

            if not self.wall_coll(new_state):
                self.state = new_state
                self.solver.t += dt
            else:
                # print("COLLISION WALL")
                wall_coll_state, coll_t = self.wall_coll_resp(new_state, self.solver.t, dt)
                self.state = wall_coll_state
                self.solver.t = coll_t

            if self.bumper_coll(self.state, other):
                # print("COLLISION BUMPER")
                self.bumper_coll_resp(self.state, other, self.solver.t, dt)

            # print(self.state)

            self.down_under()
            self.solver.set_initial_value(self.state, self.solver.t)




class Bumper(pygame.sprite.Sprite):
    # kind of like a ball, but it doesn't move
    def __init__(self, name, x, y, color=BLUE, radius=1, bounce=1):
        pygame.sprite.Sprite.__init__(self)

        if color==BLACK:
            color=GREY
        self.image = pygame.Surface([radius*2, radius*2])
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()


        pygame.draw.circle(self.image, color, (radius, radius), radius)


        self.state = np.zeros(4)
        self.state[:2] = [x,y]
        self.mass = 10
        self.radius = radius
        self.name = name
        self.bounce = bounce
        self.color = color

    # def set_radius(self, r):
    #     self.radius = r
    #     self.image = pygame.Surface([self.radius*2, self.radius*2])
    #     self.image.set_colorkey(BLACK)
    #     # self.rect = self.image.get_rect()
    #     # self.rect.x, self.rect.y = self.to_screen()
    #     print(self.state)
    #     pygame.draw.circle(self.image, self.color, (self.state[0], self.state[1]), self.radius)

    def set_pos(self, pos):
        self.state[0:2] = pos
        self.image = pygame.Surface([self.radius*2, self.radius*2])
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        self.rect.x, self.rect.y = self.to_screen()
        pygame.draw.circle(self.image, self.color, (self.state[0], self.state[1]), self.radius)

    def set_bounce(self, b):
        self.bounce = b
    def get_pos(self):
        return self.state[0:2]
    def set_vel(self, vel):
        pass
    def get_vel(self):
        return np.array([0,0])
    def get_mass(self):
        return self.mass

    def to_screen(self):
        return self.state[:2] - np.array([self.radius, self.radius])


# never got around to making the spring...
class Spring(pygame.sprite.Sprite):
    # xy is the anchor location, length is the distance from the anchor
    def __init__(self,name,x,y,length):
        pygame.sprite.Sprite.__init__(self)
        # size:
        radius = 20

        self.image = pygame.Surface([radius*2,radius*2])
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()

        pygame.draw.circle(self.image, GREEN, (radius, radius), radius)

        self.state = np.zeros(4)
        self.state[:2] = [x,y+length]
        self.rest = length
        self.name = name

    def set_pos(self, pos):
        self.state[:2] = pos

    # def update1(self, dt):




class Board:
    def __init__(self):
        self.w, self.h = window[0], window[1]
        self.objects_dict = {}
        # self.ball
        self.objects = pygame.sprite.Group()
        self.dt = 0.033

    def add_ball(self, obj):
        self.ball = obj
        self.objects.add(obj)

    def add_obj(self, obj):
        obj.rect.x, obj.rect.y = obj.to_screen()
        self.objects_dict[obj.name] = obj
        self.objects.add(obj)

    def update(self):
        obj = self.ball
        obj.update1(self.objects_dict, self.dt)
        obj.rect.x, obj.rect.y = obj.state[:2] - np.array([obj.radius, obj.radius])

        # print ('Name', obj.name)
        # print ('Position in simulation space', obj.state[:2])
        # print ('Velocity in space', obj.state[-2:])

        self.objects.update()

    def draw(self, screen):
        self.objects.draw(screen)


def main():

    # Initializing pygame
    print("---INSTRUCTIONS---")
    print("1. Press F and J to make the Blue bumpers more bouncy")
    print("2. Try not to have too much fun!")
    pygame.init()
    win_width = window[0]
    win_height = window[1]
    screen = pygame.display.set_mode((win_width, win_height))  # Top left corner is (0,0)
    pygame.display.set_caption('Pinball')

    board = Board()

    ball = Ball("ball", 1, RED, 10)
    ball.set_vel([100,90])
    ball.set_pos([100,100])
    board.add_ball(ball)

    bumper1 = Bumper("b1", 200, 200, GREY, 30)
    bumper2 = Bumper("b2", 100, 300, GREY, 30)
    bumper3 = Bumper("b3", 350, 400, GREY, 30)
    bumper4 = Bumper("b4", 200, 400, GREY, 30)
    bumper5 = Bumper("b5", 500, 800, GREY, 200)
    bumper6 = Bumper("b6", 0, 800, GREY, 200)
    bumper7 = Bumper("b7", 400, 540, GREY, 30)
    bumper8 = Bumper("b8", 515, -15, GREY, 45)
    flipperR = Bumper("fL",310, 780, BLUE, 35)
    flipperL = Bumper("fR",190, 780, BLUE, 35)

    board.add_obj(bumper1)
    board.add_obj(bumper2)
    board.add_obj(bumper3)
    board.add_obj(bumper4)
    board.add_obj(bumper5)
    board.add_obj(bumper6)
    board.add_obj(bumper7)
    board.add_obj(bumper8)
    board.add_obj(flipperL)
    board.add_obj(flipperR)

    total_frames = 30000
    iter_per_frame = 1

    frame = 0
    while frame < total_frames:
        clock.tick(30)

        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            pygame.quit()
            sys.exit(0)
        # FLIPPERS
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_f:
            # flipperL.set_pos([flipperL.state[0]-20,flipperL.state[1]])
            flipperL.set_bounce(2)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_j:
            # flipperR.set_pos([flipperR.state[0]-20,flipperR.state[1]])
            flipperR.set_bounce(2)
        elif event.type == pygame.KEYUP and event.key == pygame.K_f:
            # flipperL.set_pos([flipperL.state[0]+20,flipperL.state[1]])
            flipperL.set_bounce(1)
        elif event.type == pygame.KEYUP and event.key == pygame.K_j:
            # flipperR.set_pos([flipperR.state[0]+20,flipperR.state[1]])
            flipperR.set_bounce(1)
            # flipperL.
        else:
            pass

        board.update()
        if frame % iter_per_frame == 0:
            screen.fill(WHITE) # clear the background
            board.draw(screen)
            pygame.display.flip()
        frame += 1


    pygame.quit()

if __name__ == '__main__':
    main()
