#!/usr/bin/env python3

class RobotDimension:
    def __init__(self, diameter = 0.6, inflation = 0.5):
        self.diameter = diameter
        self.path_inflation = (self.diameter/2) + inflation