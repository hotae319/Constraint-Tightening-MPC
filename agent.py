# Import packages.
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

class Agent:
    def __init__(self, A, B, C, D, E, F, x0, W):
        self.A = A
        self.B = B        
        self.C = C
        self.D = D
        self.E = E
        self.F = F 
        self.x = x0
        self.W = W # polytope
    def update(self, u_input, w):
        self.u = u_input        
        self.x = self.A @ self.x + self.B @ self.u + w
        self.y = self.C @ self.x + self.D @ self.u
        return self.x
    def predict_z(self,E, F):
        self.z = E @ self.x + F @ self.u 
        return self.z

