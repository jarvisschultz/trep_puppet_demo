import numpy as np


class Polygon:

    def get_perim(self):
        pout = 0
        for i,p in enumerate(self.points[0:-1]):
            pout += np.linalg.norm(self.points[i+1]-p)
        pout += np.linalg.norm(self.points[0] - self.points[-1])
        return pout

class Rectangle(Polygon):
    def __init__(self, points):
        self.points = points
        self.pt1 = points[0] 
        self.pt2 = points[1]
        self.pt3 = points[2]
        self.pt4 = points[3]

    def get_area(self):
        p1 = np.linalg.norm(self.pt2-self.pt1)
        p2 = np.linalg.norm(self.pt3-self.pt2)
        return p1*p2


pts = np.array([
    [0,0],
    [0,1],
    [1,1],
    [1,0]])
