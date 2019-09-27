import pygame
from pygame.locals import QUIT
from utils.Vec2d import Vec2d


class RoadUserSprite(pygame.sprite.Sprite):
    def __init__(self, filepath, size, initial_position):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(filepath).convert_alpha()
        self.image = pygame.transform.scale(self.image, size)
        self.rect = self.image.get_rect()
        self.rect.topleft = initial_position


class CarSprite(RoadUserSprite):
    def __init__(self, initial_position):
        self.car_file = "medias/redcar.png"
        self.car_size = (200, 100)
        super(CarSprite, self).__init__(self.car_file, self.car_size, initial_position)

    def update(self, position):
        self.rect.x, self.rect.y = position.x, position.y

class HumanSprite(pygame.sprite.Sprite):
    def __init__(self, initial_position):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.Surface([30, 30])
        self.image.fill([255,255,255])
        self.rect=self.image.get_rect()
        self.rect.topleft=initial_position


if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode((640, 480), 0, 32)
    car_file = "medias/redcar.png"
    # sprite = CarSprite(car_file, (200, 100), (100, 100))

    car1 = CarSprite((0, 0))
    car2 = CarSprite((0, 300))
    human = HumanSprite((200, 200))
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                exit()
        screen.blit(car1.image, car1.rect)
        screen.blit(car2.image, car2.rect)
        screen.blit(human.image, human.rect)
        position = Vec2d(100, 200)
        car1.update(position)
        pygame.display.update()
