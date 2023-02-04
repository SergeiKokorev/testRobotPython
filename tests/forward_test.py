import sys
import os
import numpy as np

from math import radians as rad, degrees as deg


IMPORTED = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    os.pardir
))

sys.path.append(IMPORTED)

from Robot.robot import *
from ForwardKinematics import *


if __name__ == '__main__':

    robot = Robot()
    links = [
        Link(d=0.1625, teta=rad(302), a=0.0, alpha=rad(90)),
        Link(d=0.0, teta=rad(190), a=-0.425, alpha=0.0),
        Link(*[0.0, rad(59), -0.3922, 0.0]),
        Link(*[0.1333, rad(227.65), 0.0, rad(90)]),
        Link(*[0.0997, rad(120), 0.0, rad(-90)]),
        Link(*[0.0996, rad(90), 0.0, 0.0])
    ]
    
    for pos, link in enumerate(links):
        robot.addLink(link, pos)

    print(robot)
    print(robot.getBaseTransformMatrix(0, 3))

    robot.showBaseMatrix()

    print(robot.getPosition(0, 6))
    print(robot.getOrientation(0, 6))
