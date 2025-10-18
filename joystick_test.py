import pygame
pygame.init()
pygame.joystick.init()

j = pygame.joystick.Joystick(0)
j.init()
print("Joystick name:", j.get_name())

while True:
    for event in pygame.event.get():
        if (event.type == pygame.JOYAXISMOTION):
            if abs(event.value) < 0.02:
                event.value = 0
            print(event.axis, event.value)
    pygame.time.delay(200)
