import numpy as np
from math import sin,cos,radians,sqrt

# double pendulum consists of pendulum number 1 and number 2, pendulums number 2 and number 3 are connected by a spring
# all quantities are in International System of Units

class system_model():
    """
    Double pendulum consists of pendulum number 1 and number 2, pendulums number 2 and number 3 are connected by a spring.
    All quantities are in International System of Units
    """
    def __init__(self, l1=0.08, l2=0.2, l3=0.32, lk=0.14 , m1=5.0, m2=3.0, m3=1.0,
                 a1_0=30.0, a2_0=10.0, a3_0 = -30.0, v1_0 = 0.0, v2_0 = 0.0, v3_0 = 0.0, 
                 x0 = 0.1, k = 100.0, l0 = 0.05):
        self.l1=l1   # pendulum lengths
        self.l2=l2
        self.l3=l3
        self.lk=lk   # spring attachment point
        self.m1=m1      # masses
        self.m2=m2
        self.m3=m3
        self.a1_0=radians(a1_0)  # initial deviation of pendulums from the vertical
        self.a2_0=radians(a2_0)
        self.a3_0=radians(a3_0)
        self.v1_0=v1_0          # initial velocities
        self.v2_0=v2_0
        self.v3_0=v3_0
        self.x0=x0  # horizontal distance between pendulums
        self.k=k   # spring constant
        self.l0=l0  # unstretched spring length
    
    def f4(self, w):
        """ v1'=f4 """
        g=9.81
        a1=w[0]
        a2=w[1]
        a3=w[2]
        v1=w[3]
        v2=w[4]
        print(a3, a2, a1)
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
        g=9.81
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
        g=9.81
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
    def calculate(self, T = 10.0, h = 0.001):
        N=int(T/h+1)
        f=np.array([self.f1,self.f2,self.f3,self.f4,self.f5,self.f6])
        eta0=np.array([self.a1_0,self.a2_0,self.a3_0,self.v1_0,self.v2_0,self.v3_0], dtype=float)
        k1=np.array([0 for i in range(6)],dtype=float)
        k2=np.array([0 for i in range(6)],dtype=float)
        k3=np.array([0 for i in range(6)],dtype=float)
        k4=np.array([0 for i in range(6)],dtype=float)
        eta=np.array([[0 for i in range(6)] for j in range(N)],dtype=float)
        eta[0]=eta0
        for n in range(N-1):
            for i in range(6):
                k1[i]=f[i](eta[n])
                k2[i]=f[i](eta[n]+0.5*h*k1[i])
                k3[i]=f[i](eta[n]+0.5*h*k2[i])
                k4[i]=f[i](eta[n]+h*k3[i])
            eta[n+1]=eta[n]+1/6*h*(k1+2*k2+2*k3+k4)
        x1=self.l1*np.sin(eta[:,0])+self.x0
        x2=self.l1*np.sin(eta[:,0])+self.l2*np.sin(eta[:,1])+self.x0
        x3=self.l3*np.sin(eta[:,2])
        xk_0=self.lk*np.sin(eta[:,2])
        xk_1=self.l1*np.sin(eta[:,0])+(self.lk-self.l1)*np.sin(eta[:,1])+self.x0
        y1=-self.l1*np.cos(eta[:,0])
        y2=-(self.l1*np.cos(eta[:,0])+self.l2*np.cos(eta[:,1]))
        y3=-self.l3*np.cos(eta[:,2])
        yk_0=-self.lk*np.cos(eta[:,2])
        yk_1=-(self.l1*np.cos(eta[:,0])+(self.lk-self.l1)*np.cos(eta[:,1]))
        t=np.array([i*h for i in range(N)])
        return t, x1, x2, x3, xk_0, xk_1, y1, y2, y3, yk_0, yk_1
    # fig = plt.figure()
    # ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
    #                      xlim=(-0.4, 0.5), ylim=(-0.5, 0.2))
    # line1, = ax.plot([], [], 'o-', lw=2)
    # line2, = ax.plot([], [], 'o-', lw=2)
    # line3, = ax.plot([], [], 'o-', lw=2)
    # line4, = ax.plot([], [], 'm--', lw=1)
    # time_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)
    # def init():
    #     line1.set_data([], [])
    #     line2.set_data([], [])
    #     line3.set_data([], [])
    #     line4.set_data([], [])
    #     time_text.set_text('')
    #     return line1, line2, line3, line4, time_text
    # def animate(i):
    #     line1.set_data([[x1[i],x2[i]],[y1[i], y2[i]]])
    #     line2.set_data([[x0,x1[i]],[0, y1[i]]])
    #     line3.set_data([[0,x3[i]],[0, y3[i]]])
    #     line4.set_data([[xk_0[i],xk_1[i]],[yk_0[i], yk_1[i]]])
    #     time_text.set_text('Time = %.3f s' % t[i])
    #     return line1, line2, line3, line4, time_text
    # anim = animation.FuncAnimation(fig, animate, init_func=init,
    #                                frames=N, interval=h, blit=True)
    # plt.show()