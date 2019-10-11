import pygame
from utils.Vec2d import Vec2d


class RoadUserSprite(pygame.sprite.Sprite):
    def __init__(self, filepath, size, initial_position):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(filepath).convert_alpha()
        self.image = pygame.transform.scale(self.image, size)
        self.rect = self.image.get_rect()
        self.rect.topleft = initial_position
        self.ang = 0

    def update(self, position, angle):
        # self.rect.x, self.rect.y = position.x, position.y
        if self.ang != angle:
            self.image = pygame.transform.rotate(self.image, angle)
            self.rect = self.image.get_rect()
            self.ang = angle
        self.rect.center = position


class CarSprite(RoadUserSprite):
    _car_file = "medias/redcar.png"
    _car_size = (200, 100)

    def __init__(self, initial_position):
        super(CarSprite, self).__init__(self._car_file, self._car_size, initial_position)


class PedestrianSprite(RoadUserSprite):
    _pedestrian_file = "medias/pedestrian.png"
    _pedestrian_size = (50, 50)

    def __init__(self, initial_position):
        super(PedestrianSprite, self).__init__(self._pedestrian_file, self._pedestrian_size, initial_position)


if __name__ == '__main__':
    from pygame.locals import QUIT
    pygame.init()
    screen = pygame.display.set_mode((640, 480), 0, 32)

    car1 = CarSprite((0, 0))
    car2 = CarSprite((0, 300))
    human = PedestrianSprite((200, 200))

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                exit()
        screen.blit(car1.image, car1.rect)
        screen.blit(car2.image, car2.rect)
        screen.blit(human.image, human.rect)
        position = Vec2d(100, 200)
        car1.update(position, -90)
        position = Vec2d(300, 300)
        human.update(position, 180)
        pygame.display.update()
