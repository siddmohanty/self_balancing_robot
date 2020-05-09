import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions
from math import *
from pyglet.gl import *
import time
from pymunk.vec2d import Vec2d
import numpy as np

wid = 1280
hei =int(wid *(9/16))

window = pyglet.window.Window(wid,hei, "Robotics and Automation Project - Self Balancing Robot Simulation Using PID Controller", resizable=False)
options = DrawOptions()

space = pymunk.Space()
space.gravity = 0, -1000

Kp, Ki, Kd = 0.4,1.0,0
mass = 1
mass_cm = 5
radius = 10
l_shift = 300
B = 75
W = 20
hl = 200
frame = 0
max_v = 10000
acc = 40
sticky = 0  # 2000
theta_list = []

B += hl*cos(atan(W/B))
W += hl*sin(atan(W/B))
body_coor = ((wid/2)-W+l_shift,300+B)
box_coor = ((wid/2)+l_shift,300)

moment = pymunk.moment_for_circle(mass_cm, 0, radius, (0,0))
body = pymunk.Body(mass_cm, moment)
body.position = body_coor
shape = pymunk.Circle(body, radius)

space.add(body, shape)

box_size = 50
radius_ = 30
# b_moment = pymunk.moment_for_box(mass,(box_size,box_size))
b_moment = pymunk.moment_for_circle(mass,0,radius_,(0,0))
box = pymunk.Body(mass, b_moment,pymunk.Body.DYNAMIC)
box.position = box_coor
box_s = pymunk.Circle(box,radius_)
# box_s = pymunk.Poly(box,((0,0),(0,box_size),(box_size,box_size),(box_size,0)))
box.velocity=(0,0)
box.friction = 1.0
box.elasticity = 1.0
space.add(box, box_s)

segment = pymunk.Segment(space.static_body, (0,270),(window.width,270),2)
segment.body.position = 0,0
segment.friction = 1.0
segment.elasticity = 1.0
space.add(segment)

pj = pymunk.PinJoint(box, body, (0,0),(0,0))
# pj = pymunk.PinJoint(box, body, (box_size/2,box_size/2), (0,0))
space.add(pj)

theta = 0
non_rescuable = False

PID_var = {
    'I_global':0,
    'previous_error': 0
}

def maketheta():
    global theta_
    x_p = box.position[0]
    if theta < 0:
        x_p -= 40
    else:
        x_p += 25
    theta_ = pyglet.text.Label('\u03B8',
                              font_name="Times New Roman",
                              font_size=36,
                              x = x_p,
                              y = box.position[1]+80)

def makeCircle(numPoints, percent, center_coor):
    global circle
    global non_rescuable

    if non_rescuable:
        circle = pyglet.graphics.vertex_list(1, ('v2f', (0, 0)))
        return None
    if percent > 0.23:
        non_rescuable = True
        circle = pyglet.graphics.vertex_list(1, ('v2f', (0,0)))
        return None
    verts = []
    for i in range(numPoints):
        angle = radians(float(i)/numPoints * 360.0)
        if percent < 0:
            angle = -angle
        x = 50*sin(angle) + center_coor[0]
        y = 50*cos(angle) + center_coor[1]
        verts += [x,y]
        if (i/numPoints) > abs(percent):
            break
    circle = pyglet.graphics.vertex_list(int(len(verts)/2), ('v2f', verts))


def PID(theta):
    error = (360*theta)/pi
    P = Kp*error
    PID_var['I_global'] += Ki*error
    D = Kd * (error - PID_var['previous_error'])
    PID_var['previous_error'] = error
    return sum([P,PID_var['I_global'],D])

def update(dt):
    global theta
    global frame
    frame += 1
    space.step(dt)
    o_a = (body.position[0] - (box.position[0]))/(body.position[1] - (box.position[1]+15))
    theta = atan(o_a)
    theta_list.append(theta)
    np.savetxt('theta_list.txt',np.array(theta_list))
    if theta > 1.4:
        v = 0
        print("Fallen")

    else:
        v = PID(theta)
    if abs(v) < sticky:
        v = 0


    if v > box.velocity[0]:
        box.velocity = [box.velocity[0]+acc,0]
    elif v < box.velocity[0]:
        box.velocity = [box.velocity[0]-acc,0]
    # box.velocity = [v,0]
    makeCircle(100, theta/(2*pi), [box.position[0], box.position[1]+15])
    maketheta()
    # time.sleep(1)

makeCircle(100, 0, [300, 300])
maketheta()

@window.event
def on_draw():
    window.clear()
    global circle
    circle.draw(GL_LINE_STRIP)
    space.debug_draw(options)
    theta_.draw()
    for i in range(10):
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ("v2f", (box.position[0],box.position[1]+30+i*10,box.position[0],box.position[1]+35+i*10)),
                             )
    # pyglet.image.get_buffer_manager().get_color_buffer().save(str(frame) + '.png')
    # print(pyglet.clock.tick())

if __name__ == "__main__":
    pyglet.clock.schedule_interval(update, 1.0/60)
    pyglet.app.run()
