import pygame

pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()
print(f'Number of joysticks: {joystick_count}')

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f'Joystick Name: {joystick.get_name()}')
    print(f'Number of Axes: {joystick.get_numaxes()}')
    print(f'Number of Buttons: {joystick.get_numbuttons()}')