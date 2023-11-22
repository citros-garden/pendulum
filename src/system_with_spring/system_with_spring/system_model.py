import numpy as np
from math import sin,cos,radians,sqrt
import scipy.integrate as solver

class system_model():
    """
    Double pendulum consists of pendulum number 1 and number 2, pendulums number 2 and number 3 are connected by a spring.
    All quantities are in International System of Units
    """
    g=9.81

    def __init__(self, l1=0.08, l2=0.2, l3=0.32, lk=0.14 , m1=5.0, m2=3.0, m3=1.0,
                 a1_0=30.0, a2_0=10.0, a3_0 = -30.0, v1_0 = 0.0, v2_0 = 0.0, v3_0 = 0.0, 
                 x0 = 0.1, k = 100.0, l0 = 0.05):
        self.l1=l1   # pendulum lengths
        self.l2=l2
        self.l3=l3
        self.lk=lk   # spring attachment point
        self.m1=m1   # masses
        self.m2=m2
        self.m3=m3
        self.a1_0=radians(a1_0)  # initial deviation of pendulums from the vertical
        self.a2_0=radians(a2_0)
        self.a3_0=radians(a3_0)
        self.v1_0=radians(v1_0)       # initial velocities
        self.v2_0=radians(v2_0)
        self.v3_0=radians(v3_0)
        self.x0=x0  # horizontal distance between pendulums
        self.k=k   # spring constant
        self.l0=l0  # unstretched spring length
    
    def f4(self, w):
        """ v1'=f4 """
        g = self.g
        a1=w[0]
        a2=w[1]
        a3=w[2]
        v1=w[3]
        v2=w[4]
        dy=self.lk*(cos(a3)-cos(a2))+self.l1*(cos(a2)-cos(a1))
        dx=self.l0+self.lk*(sin(a2)-sin(a3))+self.l1*(sin(a1)-sin(a2))
        dlp=self.l0-sqrt(dy**2+dx**2)
        d1=-(dy*self.l1*sin(a1)+dx*self.l1*cos(a1))/(self.l0-dlp)
        d2=-(dy*(sin(a2)*(self.lk-self.l1))+dx*cos(a2)*(self.lk-self.l1))/(self.l0-dlp)
        s=(self.m2*self.l1**2*sin(a1-a2)*cos(a1-a2)*v1**2+self.m2*self.l1*self.l2*sin(a1-a2)*v2**2-self.m2*g*self.l1*sin(a2)*cos(a1-a2)+
        g*self.l1*sin(a1)*(self.m1+self.m2)-0.5*self.l1/self.l2*self.k*2*dlp*cos(a1-a2)*d2+
        0.5*self.k*2*dlp*d1)/(self.m2*self.l1**2*(cos(a1-a2))**2-(self.m1+self.m2)*self.l1**2)
        return(s)
    def f5(self, w):
        """ v2'=f5 """
        g = self.g
        a1=w[0]
        a2=w[1]
        a3=w[2]
        v1=w[3]
        v2=w[4]
        dy=self.lk*(cos(a3)-cos(a2))+self.l1*(cos(a2)-cos(a1))
        dx=self.l0+self.lk*(sin(a2)-sin(a3))+self.l1*(sin(a1)-sin(a2))
        dlp=self.l0-sqrt(dy**2+dx**2)
        d1=-(dy*self.l1*sin(a1)+dx*self.l1*cos(a1))/(self.l0-dlp)
        d2=-(dy*(sin(a2)*(self.lk-self.l1))+dx*cos(a2)*(self.lk-self.l1))/(self.l0-dlp)
        s=(-self.l2**2*self.m2**2/(self.m1+self.m2)*cos(a1-a2)*sin(a1-a2)*v2**2-self.m2*self.l1*self.l2*sin(a1-a2)*v1**2+self.m2*g*self.l2*sin(a2)-
        g*self.l2*sin(a1)*self.m2*cos(a1-a2)+0.5*self.k*2*dlp*d2-
        0.5*self.k*2*dlp*d1*self.l2/self.l1*self.m2/(self.m1+self.m2)*cos(a1-a2))/(self.l2**2*self.m2**2*(cos(a1-a2))**2/(self.m1+self.m2)-self.m2*self.l2**2)
        return(s)
    def f6(self, w):
        """ v3'=f6 """
        g = self.g
        a1=w[0]
        a2=w[1]
        a3=w[2]
        dy=self.lk*(cos(a3)-cos(a2))+self.l1*(cos(a2)-cos(a1))
        dx=self.l0+self.lk*(sin(a2)-sin(a3))+self.l1*(sin(a1)-sin(a2))
        dlp=self.l0-sqrt(dy**2+dx**2)
        d3=-(-dy*self.lk*sin(a3)-dx*self.lk*cos(a3))/(self.l0-dlp)
        s=(-self.m3*g*self.l3*sin(a3)-0.5*self.k*2*dlp*d3)/(self.m3*self.l3**2)
        return(s)
    def f1(self, w):
        """ a1'=f1 """
        return(w[3])
    def f2(self, w):
        """ a2'=f2 """
        return(w[4])
    def f3(self, w):
        """ a3'=f3 """
        return(w[5])
    def fun(self, t, w):
        return [self.f1(w), self.f2(w), self.f3(w), self.f4(w), self.f5(w), self.f6(w)]
    def calculate(self, T = 10.0, h = 0.01):
        N=int(T/h+1)
        t=np.array([i*h for i in range(N)])
        eta0=np.array([self.a1_0,self.a2_0,self.a3_0,self.v1_0,self.v2_0,self.v3_0], dtype=float)
        t_span = [0.0, T]
        res = solver.solve_ivp(self.fun, t_span, eta0, t_eval= t)
        x1=self.l1*np.sin(res.y[0])+self.x0
        x2=self.l1*np.sin(res.y[0])+self.l2*np.sin(res.y[1])+self.x0
        x3=self.l3*np.sin(res.y[2])
        xk_0=self.lk*np.sin(res.y[2])
        xk_1=self.l1*np.sin(res.y[0])+(self.lk-self.l1)*np.sin(res.y[1])+self.x0
        y1=-self.l1*np.cos(res.y[0])
        y2=-(self.l1*np.cos(res.y[0])+self.l2*np.cos(res.y[1]))
        y3=-self.l3*np.cos(res.y[2])
        yk_0=-self.lk*np.cos(res.y[2])
        yk_1=-(self.l1*np.cos(res.y[0])+(self.lk-self.l1)*np.cos(res.y[1]))
        return t, x1, x2, x3, xk_0, xk_1, y1, y2, y3, yk_0, yk_1