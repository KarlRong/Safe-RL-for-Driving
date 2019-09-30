from sprites import CarSprite, PedestrianSprite
from utils.Vec2d import Vec2d


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
        self.heading, self.timeNode, self.nTraj = self.getLength(path)
        self.iTraj = 0
        self.position = path[0]
        self.time = 0
        self.vel = velocity
        self.angle = 0
        if isCar:
            self.sprite = CarSprite(path[0])
        else:
            self.sprite = PedestrianSprite(path[0])

    def handlePath(self, path):
        n = len(path)
        headings = [Vec2d(0, 0)]
        timeNode = [0]
        if n > 1:
            L = 0
            for i in range(0, n - 2):
                head = path[i + 1] - path[i]
                headings.append(head)
                l = head.length
                t = l / self.vel
                L += l
                timeNode.append(timeNode[i] + t)
        return headings, timeNode, n

    def processPosition(self, t):
        if self.iTraj < self.nTraj - 1 && t > self.timeNode[self.iTraj + 1]:
            self.iTraj += 1
        

    def process(self, time_passed):
        self.iTraj += 1
        self.position = self.traj[self.iTraj]
        self.sprite.update(self.position, self.angle)

    def render(self, screen):
        screen.blit(self.sprite.image, self.sprite.rect)


if __name__ == '__main__':
    import pygame
    from pygame.locals import QUIT
    import time

    pygame.init()
    screen = pygame.display.set_mode((640, 480), 0, 32)

    traj1 = [Vec2d(100, 100), Vec2d(200, 100), Vec2d(300, 100), Vec2d(400, 100), Vec2d(400, 200), Vec2d(400, 300)]
    velocity = 10
    car = RoadUser(traj1, velocity)

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                exit()
        car.render(screen)
        car.process()
        pygame.display.update()
        time.sleep(1)
