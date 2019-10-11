from sprites import CarSprite, PedestrianSprite
from utils.Vec2d import Vec2d
import numpy as np


class Entity:
    def __init__(self):
        pass

    def process(self, *args):
        pass

    def render(self, *args):
        pass


class Agent(Entity):
    def __init__(self):
        super(Agent, self).__init__()

    def process(self, *args):
        pass

    def render(self, *args):
        pass


class RoadUser(Entity):
    def __init__(self, path, vel, isCar=True):
        super(Entity, self).__init__()
        self.path = path
        self.iTraj = 0
        self.time = 0
        self.vel = velocity

        self.headings, self.timeNode, self.nTraj = self.handlePath(path)
        self.position, self.angle = self.getPosition(0)
        if isCar:
            self.sprite = CarSprite(path[0])
        else:
            self.sprite = PedestrianSprite(path[0])

    def handlePath(self, path):
        n = len(path)
        timeNode = [0]
        if n > 1:
            L = 0
            headings = []
            for i in range(0, n - 1):
                head = path[i + 1] - path[i]
                l = head.length
                t = l / self.vel
                L += l
                timeNode.append(timeNode[i] + t)
                head = head / head.length
                headings.append(head)
        else:
            headings = [Vec2d(0, 0)]
        return headings, timeNode, n

    def getPosition(self, t):
        i = np.searchsorted(self.timeNode, t)
        if i >= self.nTraj - 1:
            pos = self.path[-1]
            ang = self.headings[-1].angle
        else:
            gap = t - self.timeNode[i]
            ang = self.headings[i].angle
            pos = self.path[i] + (gap * self.vel) * self.headings[i]
        return pos, ang

    def getCurrentPos(self, time_passed):
        self.time += time_passed
        if self.iTraj == self.nTraj - 1:
            pos = self.path[-1]
            ang = self.headings[-1].angle
        elif self.time > self.timeNode[-1]:
            self.iTraj += 1
            pos = self.path[-1]
            ang = self.headings[-1].angle
        else:
            if self.time > self.timeNode[self.iTraj + 1]:
                self.iTraj += 1
            gap = self.time - self.timeNode[self.iTraj]
            ang = self.headings[self.iTraj].angle
            pos = self.path[self.iTraj] + (gap * self.vel) * self.headings[self.iTraj]
        ang = -ang
        return pos, ang

    def process(self, time_passed):
        self.position, self.angle = self.getCurrentPos(time_passed)
        self.sprite.update(self.position, self.angle)

    def render(self, screen):
        screen.blit(self.sprite.image, self.sprite.rect)


if __name__ == '__main__':
    import pygame
    from pygame.locals import QUIT

    pygame.init()
    screen_size = (640, 480)
    screen = pygame.display.set_mode(screen_size, 0, 32)
    background = pygame.surface.Surface(screen_size)
    background.fill((204, 196, 138))
    clock = pygame.time.Clock()

    traj1 = [Vec2d(100, 100), Vec2d(200, 100), Vec2d(300, 100), Vec2d(400, 100), Vec2d(400, 200), Vec2d(400, 300)]
    velocity = 50
    car = RoadUser(traj1, velocity)

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                exit()
        time_passed = clock.tick(30) / 1000
        print("time_passed", time_passed)
        car.process(time_passed)
        screen.blit(background, (0, 0))
        car.render(screen)
        pygame.display.update()
        # time.sleep(1)
