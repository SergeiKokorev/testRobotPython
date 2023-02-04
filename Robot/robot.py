import numpy as np
from math import sin, cos, atan, atan2, degrees as deg
 
from abc import ABCMeta, abstractmethod, abstractproperty
from dataclasses import dataclass


@dataclass
class EulerAngles:
    alpha: float
    beta: float
    gamma: float

    @property
    def angles(self) -> list:
        return [self.alpha, self.beta, self.gamma]

    def __repr__(self):
        angles = str([round(deg(a), 4) for a in self.angles])[1:-1]
        angles += '\n'
        angles += str([round(a, 4) for a in self.angles])[1:-1]
        return angles

@dataclass
class AbstractLink(metaclass=ABCMeta):
    
    @abstractproperty
    def RotX(self):
        pass

    @abstractproperty
    def RotZ(self):
        pass

    @abstractproperty
    def TrX(self):
        pass
    
    @abstractproperty
    def TrZ(self):
        pass

    @abstractmethod
    def getTransformMatrix(self):
        pass


class AbstractRobot(metaclass=ABCMeta):
    links = []

    @abstractmethod
    def addLink(self):
        pass

    @abstractmethod
    def delLink(self):
        pass

    @abstractmethod
    def getLink(self):
        pass

    @abstractproperty
    def BaseRFTransformMatrix(self):
        pass

    @abstractmethod
    def getBaseTransformMatrix(self):
        pass

    @abstractmethod
    def getPosition(self):
        pass

    @abstractmethod
    def getOrientation(self):
        pass

    def __repr__(self):
        show_links = ''
        for i, link in enumerate(self.links, 1):
            show_links += f'{i}:\t{link.__repr__()}'
        return show_links


@dataclass
class Link(AbstractLink):
    d: float | int
    teta: float | int
    a: float | int
    alpha: float | int

    @property
    def RotX(self):
        return np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, cos(self.alpha), -sin(self.alpha), 0.0],
            [0.0, sin(self.alpha), cos(self.alpha), 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    @property
    def RotZ(self):
        return np.array([
            [cos(self.teta), -sin(self.teta), 0.0, 0.0],
            [sin(self.teta), cos(self.teta), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    @property
    def TrX(self):
        return np.array([
            [1.0, 0.0, 0.0, self.a],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
    
    @property
    def TrZ(self):
        return np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, self.d],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def getTransformMatrix(self):
        return self.RotZ @ self.TrZ @ self.TrX @ self.RotX

    def __repr__(self):
        return f'{self.__class__.__name__}({round(self.d, 4)}, ' \
            f'{round(self.teta, 4)}, {round(self.a, 4)}, ' \
            f'{round(self.alpha, 4)})\n'


class Robot(AbstractRobot):

    def addLink(self, link: Link, pos: int) -> None:
        if not (isinstance(link, Link) and isinstance(pos, int)):
            msg = f'Given wrong arguments: link {type(link).__name__};'
            msg += f'pos: {type(pos).__name__}'
            raise TypeError(msg)
        
        self.links.insert(pos, link)

    def delLink(self, link: Link, pos: int) -> None:
        if not isinstance(link, Link) and isinstance(pos, int):
            msg = f'Given wrong arguments: link {type(link).__name__};'
            msg += f'pos: {type(pos).__name__}'
            raise TypeError(msg)
        try:
            return self.links.pop(pos)
        except:
            raise IndexError('List index out of range.')

    def getLink(self, pos: int) -> Link:
        try:
            return self.links[pos]
        except IndexError:
            raise IndexError(f'There is no link with position {pos}')

    @property
    def BaseRFTransformMatrix(self) -> np.ndarray:
        base = np.eye(4)
        for link in self.links:
            base = base @ link.getTransformMatrix()
        return base

    def getBaseTransformMatrix(self, pos1: int, pos2: int) -> np.ndarray:
        
        try:
            links = self.links[pos1:pos2]
            base = np.eye(4)
            for link in links:
                base = base @ link.getTransformMatrix()
            return base
        except IndexError:
            msg = 'pos1 must be greater than pos2.\n'
            msg += 'Both pos1 and pos2 must be integer type.\n'
            msg += f'Given args pos1 {pos1} and pos2 {pos2}'
            raise IndexError(msg)

    def getPosition(self, pos1: int, pos2: int) -> list:
        if not (isinstance(pos1, int) and isinstance(pos2, int)):
            msg = f'pos must be integer type. Given {type(pos).__name__}'
            raise TypeError(msg)

        return [i[0] for i in self.getBaseTransformMatrix(pos1, pos2)[:-1, -1:]]

    def getOrientation(self, pos1: int, pos2: int) -> EulerAngles:
        if not (isinstance(pos1, int) and isinstance(pos2, int)):
            msg = f'pos must be integer type. Given {type(pos).__name__}'
            raise TypeError(msg)
        ea = EulerAngles(0.0, 0.0, 0.0)
        m = self.getBaseTransformMatrix(pos1, pos2)
        print(m)
        ea.beta = atan2(-m[2, 0], (m[0, 0]** 2 + m[1, 0] ** 2) ** 0.5)
        ea.alpha = atan2(m[1, 0] / cos(ea.beta), m[0, 0] / cos(ea.beta))
        ea.gamma = atan2(m[2, 1] / cos(ea.beta), m[2, 2] / cos(ea.beta))
        return ea

    def showBaseMatrix(self) -> None:
        for el in self.BaseRFTransformMatrix:
            print([round(e, 4) for e in el])
