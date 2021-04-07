import pygame
import numpy as np

import pymunk
import pymunk.pygame_util

from Reel import Reel, ReactiveReel
import values

FPS = 60

pygame.init()
screen = pygame.display.set_mode((1200, 800))
clock = pygame.time.Clock()

space = pymunk.Space()
space.gravity = (0.0, 900.0)
space.damping = 0.1
draw_options = pymunk.pygame_util.DrawOptions(screen)

phone = pymunk.Body()
phone.position = 200, 800-370
# phone_shape = pymunk.Poly(phone, ((10, 2), (-10, 2), (-10, -2), (10, -2)))
phone_shape = pymunk.Segment(phone, (-10, 0), (10, 0), 2)
phone_shape.density = 2
phone_shape.elasticity = 0.8
phone_shape.friction = 0.1
space.add(phone, phone_shape)

floor = pymunk.Body(body_type=pymunk.Body.STATIC)
floor.position = 1200/2, 800
floor_shape = pymunk.Segment(floor, (-600, 0), (600, 0), 20)
floor_shape.elasticity = 0.3
floor_shape.friction = 0.4
space.add(floor, floor_shape)

chair_height = 350
chair_width = 300
chair = pymunk.Body(body_type=pymunk.Body.STATIC)
chair.position = 100, 800-chair_height
chair_shape = pymunk.Poly(chair, ((-chair_width/2, 0), (-chair_width/2, chair_height), (chair_width/2, chair_height), (chair_width/2, 0)))
chair_shape.elasticity = 0.3
chair_shape.friction = 0.4
space.add(chair, chair_shape)


auto_reel = ReactiveReel(space, 300, 400, phone, screen=screen, tether_offset=(-5, 0), distance_cutoff=200)
applied = False
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit()
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            exit()

    screen.fill(pygame.Color("white"))

    mouse_pos = pygame.mouse.get_pos()

    # if pygame.time.get_ticks() > 3*1000:
    #     # reel_in(spool, phone)
    #     reel_in_v2(phone)

    if pygame.time.get_ticks()  > 2*1000 and not applied:
        applied = True
        phone.apply_impulse_at_local_point((40000, -9000))

    auto_reel.step()
    space.step(1.0 / FPS)

    space.debug_draw(draw_options)
    pygame.display.flip()

    clock.tick(FPS)
    pygame.display.set_caption(f"fps: {round(clock.get_fps())}")
