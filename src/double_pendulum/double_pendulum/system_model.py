import numpy as np
from math import sin,cos,radians,sqrt
import scipy.integrate as solver

class system_model():
    """
    Double pendulum consists of pendulum number 1 and number 2.
    All quantities are in International System of Units.
    """
    g=9.81

    def __init__(self, l1=0.08, l2=0.2, m1=5.0, m2=3.0,
                 a1_0=30.0, a2_0=10.0, v1_0 = 0.0, v2_0 = 0.0):
        self.l1=l1   # pendulum lengths
        self.l2=l2
        self.m1=m1      # masses
        self.m2=m2
        self.a1_0=radians(a1_0)  # initial deviation of pendulums from the vertical
        self.a2_0=radians(a2_0)
        self.v1_0=radians(v1_0)          # initial velocities
        self.v2_0=radians(v2_0)
    
    def f3(self, w):
        """ v1'=f3 """
        g = self.g
        a1=w[0]
        a2=w[1]
        v1=w[2]
        v2=w[3]
        delta = a1 - a2
        s_num = -v2**2*sin(delta)*self.l2*self.m2 - (self.m1+self.m2)*g*sin(a1) - v1**2*self.l1*sin(delta)*self.m2*cos(delta)+g*self.m2*cos(delta)*sin(a2)
        s_den = self.l1*(self.m1+self.m2*sin(delta)*sin(delta))
        s = s_num/s_den
        return(s)
    def f4(self, w):
        """ v2'=f4 """
        g = self.g
        a1=w[0]
        a2=w[1]
        v1=w[2]
        v2=w[3]
        delta = a1 - a2
        s_num = -v2**2*self.m2*self.l2*cos(delta)*sin(delta)-g*(self.m1+self.m2)*sin(a1)*cos(delta)-v1**2*sin(delta)*self.l1*(self.m1+self.m2)+g*sin(a2)*(self.m1+self.m2)
        s_den = -self.l2*(self.m1+self.m2*sin(delta)*sin(delta))
        s = s_num/s_den
        return(s)
    def f1(self, w):
        """ a1'=f1 """
        return(w[2])
    def f2(self, w):
        """ a2'=f2 """
        return(w[3])
    def fun(self, t, w):
        return [self.f1(w), self.f2(w), self.f3(w), self.f4(w)]

    def calculate(self, T = 10.0, h = 0.01):
        N=int(T/h+1)
        t=np.array([i*h for i in range(N)])
        eta0=np.array([self.a1_0,self.a2_0,self.v1_0,self.v2_0], dtype=float)
        t_span = [0.0, T]
        res = solver.solve_ivp(self.fun, t_span, eta0, t_eval= t)
        x1=self.l1*np.sin(res.y[0])
        x2=self.l1*np.sin(res.y[0])+self.l2*np.sin(res.y[1])
        y1=-self.l1*np.cos(res.y[0])
        y2=-(self.l1*np.cos(res.y[0])+self.l2*np.cos(res.y[1]))

        return t, x1, x2, y1, y2