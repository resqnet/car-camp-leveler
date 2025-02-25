# vector3d.py

from math import sqrt, degrees, acos

class Vector3d:
    def __init__(self, get_x, get_y, get_z):
        self._get_x = get_x
        self._get_y = get_y
        self._get_z = get_z
        self._ivector = [0, 0, 0]

    @property
    def x(self):
        return self._ivector[0]

    @property
    def y(self):
        return self._ivector[1]

    @property
    def z(self):
        return self._ivector[2]

    @property
    def magnitude(self):
        return sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def unit(self):
        mag = self.magnitude
        return Vector3d(lambda: self.x / mag,
                        lambda: self.y / mag,
                        lambda: self.z / mag)

    def dot(self, vector):
        return self.x * vector.x + self.y * vector.y + self.z * vector.z

    def angle(self, vector):
        return degrees(acos(self.dot(vector) / (self.magnitude * vector.magnitude)))
