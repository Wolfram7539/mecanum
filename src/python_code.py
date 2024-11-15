'''
w1----------w0            Vy
|           |            |    
|           |            |    
|           |  H         |------ Vx    ↶ w
|           |
|           |
w3----------w2
      L
'''
import numpy as np
class MECANUM():
    
    def __init__(self,_r,_H,_L):
        self.r = _r
        self.H = _H
        self.L = _L
        self.output = np.zeros(4)
    def __call__(self):
        print(self.r,self.H,self.L,self.output)
        
    def calucurate(self,vx,vy,w,conv = False):
         self.output[0] = (1/self.r) * (vy-vx+(self.H+self.L)*w)
         self.output[1] = (1/self.r) * (vy+vx-(self.H+self.L)*w)
         self.output[2] = (1/self.r) * (vy+vx+(self.H+self.L)*w)
         self.output[3] = (1/self.r) * (vy-vx-(self.H+self.L)*w)
         if conv:
             self.output *= 60 / ( 2 * np.pi * self.r)
         return self.output
     
    def vector_mecanum(self,vector,theta):
        self.theta = np.deg2rad(theta)
        self.rotateMat = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)],
    ])
        
        self.nvector = np.dot(vector,self.rotateMat)
        print(self.nvector[0])
        return self.calucurate(self.nvector[0],self.nvector[1],0)
    