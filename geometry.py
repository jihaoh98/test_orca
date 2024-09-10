import numpy as np
from math import tan, atan, pi, cos, sin, sqrt, fabs
from utils import trueMod


class Point:
    """ Define a point in 2D space """
    def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
        self.x = x
        self.y = y

    def init_from_point(self, p: 'Point') -> None:
        """ initialize with a point """
        self.x = p.x
        self.y = p.y

    def init_from_vector(self, vec: 'Vector') -> None:
        """ initialize with a vector """
        self.x = vec.x
        self.y = vec.y
    
    def __add__(self, p: 'Point') -> 'Point':
        return Point(self.x + p.x, self.y + p.y)
    
    def __sub__(self, p: 'Point'):
        return Point(self.x - p.x, self.y - p.y)
    
    def __mul__(self, scale):
        return Point(self.x * scale, self.y * scale)
    
    def __truediv__(self, scale):
        return Point(self.x / scale, self.y / scale)
    
    def __eq__(self, p: 'Point') -> bool:
        return self.x == p.x and self.y == p.y
    
    def __ne__(self, p: 'Point') -> bool:
        return not self.__eq__(p)
    
    def __str__(self) -> str:
        return f"Point: (x:= {self.x}, y:= {self.y})"
    
    def from_point(self, p: 'Point') -> 'Vector':
        return Vector(self.x - p.x, self.y - p.y)
    
    def projection_ontoline(self, l: 'Line') -> 'Point':
        """ project point onto a line """
        if (l.isVertical()):
            return Point(l.xIntercept, self.y)
        else:
            denom = l.slope ** 2 + 1.0
            res_x = (self.x + l.slope * (self.y - l.yIntercept)) / denom
            res_y = (l.slope * (self.x + l.slope * self.y) + l.yIntercept) / denom
            return Point(res_x, res_y)


class Vector:
    """ Define a vector in 2D space """
    def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
        self.x = x
        self.y = y
        
    def init_from_vector(self, vec: 'Vector') -> None:
        self.x = vec.x
        self.y = vec.y
        
    def init_from_ont_point(self, p: 'Point') -> None:
        self.x = p.x
        self.y = p.y
    
    def init_from_points(self, p1: 'Point', p2: 'Point') -> None:
        self.x = p2.x - p1.x
        self.y = p2.y - p1.y
        
    def init_from_angle(self, angle: float) -> None:
        angle = trueMod(angle, 2 * pi)
        self.x = cos(angle)
        self.y = sin(angle)
        
    def get_scaled_vector(self, scale: float) -> None:
        return Vector(self.x * scale, self.y * scale)
        
    def __add__(self, vec: 'Vector') -> 'Vector':    
        return Vector(self.x + vec.x, self.y + vec.y)
    
    def __sub__(self, vec: 'Vector') -> 'Vector':
        return Vector(self.x - vec.x, self.y - vec.y)
    
    def __mul__(self, vec: 'Vector') -> 'float':
        return self.x * vec.x + self.y * vec.y
 
    def __truediv__(self, scale: float) -> 'Vector':
        return Vector(self.x / scale, self.y / scale)
    
    def __eq__(self, vec: 'Vector') -> bool:
        return self.x == vec.x and self.y == vec.y
    
    def __ne__(self, vec: 'Vector') -> bool:
        return not self.__eq__(vec)
    
    def __str__(self) -> str:
        return f"Vector: (x:= {self.x}, y:= {self.y})"
    
    def getXangle(self) -> float:
        if self.norm() == 0.0:
            return 0.0
        elif self.x == 0.0:
            if self.y > 0:
                return pi / 2
            else:
                return 3 * pi / 2
        else:
            angle = atan(self.y / self.x)
            if self.x < 0:
                angle += pi
            return trueMod(angle, 2 * pi)
       
    def norm(self) -> float:
        return sqrt(self.x ** 2 + self.y ** 2)
    
    def norm_equal(self, scale: float) -> bool:
        return self.norm() == scale
            
    def normalize(self, new_norm = 1.0) -> None:
        cur_norm = self.norm()
        if cur_norm != 0.0:
            self.x *= new_norm / cur_norm
            self.y *= new_norm / cur_norm
        
    def limitNorm(self, max_norm = 1.0) -> None:
        if self.norm() > max_norm:
            self.normalize(max_norm)
            
    def projectToVector(self, vec: 'Vector') -> 'Vector':
        """ project vector onto another vector """
        new_vec = Vector()
        new_vec.init_from_vector(vec)
        if vec.norm() != 0.0:
            new_vec.normalize(self * vec / vec.norm())
        return new_vec
    
    def getRotatedVector(self, angle: float) -> 'Vector':
        """ obtain a vector by counterclockwise rotation """
        new_vec = Vector()
        new_vec.init_from_angle(self.getXangle() + angle)
        # new_vec.normalize(self.norm())
        return new_vec
    
    def angleFrom(self, vec: 'Vector') -> float:
        return self.getXangle() - vec.getXangle()
            

class Line:
    """
    Define a line in 2D space
    ################################
    #  y = slope * x + yIntercept  #
    ################################
    in case of a vertical line, the slope is set as np.inf
    """
    def __init__(self, slope_ = 0.0, intercept_ = 0.0) -> None:
        if slope_ == -np.inf:
            self.slope = np.inf
        else:
            self.slope = slope_
        
        if self.isVertical():
            self.xIntercept = intercept_
            self.yIntercept = 0.0
        else:
            self.yIntercept = intercept_
            if self.isHorizontal():
                self.xIntercept = 0.0
            else:
                self.xIntercept = -intercept_ / slope_
            
    def init_from_angle(self, angle) -> None:
        angle = trueMod(angle, pi)
        if angle == pi / 2:
            self.slope = np.inf
        else:
            self.slope = tan(angle)

    def init_from_points(self, p1: 'Point', p2: 'Point') -> None:
        if p1.x == p2.x and p1.y != p2.y:
            self.slope = np.inf
            self.xIntercept = p1.x
            self.yIntercept = 0.0
        else:
            if p1 == p2:
                self.slope = 0.0
            else:
                self.slope = (p2.y - p1.y) / (p2.x - p1.x)
                
            self.yIntercept = p1.y - self.slope * p1.x
            if self.isHorizontal():
                self.xIntercept = 0.0
            else:
                self.xIntercept = -self.yIntercept / self.slope

    def init_from_one_point(self, p: 'Point') -> None:
        self.init_from_points(Point(), p)

    def init_from_line(self, l: 'Line') -> None:
        self.slope = l.slope
        self.xIntercept = l.xIntercept
        self.yIntercept = l.yIntercept
    
    def init_from_line_and_point(self, l: 'Line', p: 'Point') -> None:
        """ construct a line perpendicular to l and passing through p """
        if l.isHorizontal():
            self.slope = np.inf
            self.xIntercept = p.x
            self.yIntercept = 0.0
        else:
            if l.isVertical():
                self.slope = 0.0
            else:
                self.slope = -1.0 / l.slope

            self.yIntercept = p.y - self.slope * p.x
            if self.isHorizontal():
                self.xIntercept = 0.0
            else:
                self.xIntercept = -self.yIntercept / self.slope
        
    def isHorizontal(self) -> bool:
        return self.slope == 0.0

    def isVertical(self) -> bool:
        return self.slope == np.inf
    
    def get_slope_angle(self) -> float:
        return trueMod(atan(self.slope), pi)
    
    def get_rotated_line(self, angle: float) -> 'Line':
        """ obtain a line by counterclockwise rotation """
        new_line = Line()
        new_line.init_from_angle(self.get_slope_angle() + angle)
        return new_line
    
    def __eq__(self, l: 'Line') -> bool:
        if self.slope != l.slope:
            return False
        if self.isVertical():
            return self.xIntercept == l.xIntercept
        else:
            return self.yIntercept == l.yIntercept

    def __ne__(self, l: 'Line') -> bool:
        return not self.__eq__(l)
    
    def __str__(self) -> str:
        return f"Line: (slope:= {self.slope}, xIntercept:= {self.xIntercept}, yIntercept:= {self.yIntercept})"
    
    def isparallel(self, l: 'Line') -> bool:
        return self.slope == l.slope
    
    def contains_point(self, p: 'Point') -> bool:
        if self.isVertical():
            return p.x == self.xIntercept
        else:
            return p.y == self.slope * p.x + self.yIntercept
        
    def getPointatX(self, x: float) -> 'Point':
        if self.isVertical():
            print("Error: vertical line")
            return Point(x, 0.0)
        else:
            return Point(x, self.slope * x + self.yIntercept)
        
    def getPointatY(self, y: float) -> 'Point':
        if self.isHorizontal():
            print("Error: horizontal line")
            return Point(0.0, y)
        elif self.isVertical():
            return Point(self.xIntercept, y)
        else:
            return Point((y - self.yIntercept) / self.slope, y)
        
    def getIntersectionPoint(self, l: 'Line') -> 'Point':
        if self.isparallel(l):
            print("Error: lines are parallel")
            return Point()
        elif self.isVertical():
            return l.getPointatX(self.xIntercept)
        elif l.isVertical():
            return self.getPointatX(l.xIntercept)
        else:
            cur_x = (l.yIntercept - self.yIntercept) / (self.slope - l.slope)
            return self.getPointatX(cur_x)
        

class HalfPlane:
    """
    Define a half plane in 2D space
    Attributes:
        - line: the boundary line of the half plane
        - point: a point in the half plane
        - vector: a vector pointing to the outside of the half plane
    """
    def __init__(self) -> None:
        """ Define by the equation y >= 0 """
        self.normalPosition = Point()
        self.normalVector = Vector(0.0, 1.0)
        self.boundLine = Line()
        
    def __init__(self, p: 'Point', ver: 'Vector') -> None:
        """ Define a halfplane by a point and a vector """
        self.normalPosition = Point(p.x, p.y)
        if ver.norm() == 0.0:
            self.normalVector = Vector(0.0, 1.0)
        else:
            self.normalVector = Vector(ver.x, ver.y)
        
        slope = None
        intercept = None
        if self.normalVector.y == 0.0:
            slope = np.inf
            intercept = self.normalPosition.x
        else:
            slope = -self.normalVector.x / self.normalVector.y
            intercept = self.normalPosition.y - slope * self.normalPosition.x
        
        self.boundLine = Line(slope, intercept)
    
    def init_from_halfplane(self, hp: 'HalfPlane') -> None:
        self.normalPosition.init_from_point(hp.normalPosition)
        self.normalVector.init_from_vector(hp.normalVector)
        self.boundLine.init_from_line(hp.boundLine)
        
    def contains_point(self, p: 'Point') -> bool:
        """ if a point is contained in the half plane: aTx >= b """
        return p.from_point(self.normalPosition) * self.normalVector >= 0.0
    
    def __eq__(self, hp: 'HalfPlane') -> bool:
        return self.normalPosition == hp.normalPosition and \
            self.normalVector == hp.normalVector
            
    def __ne__(self, hp: 'HalfPlane') -> bool:
        return not self.__eq__(hp)
    
    def __str__(self) -> str:
        return f"HalfPlane: (normalPosition:= {self.normalPosition}, normalVector:= {self.normalVector})"
        
