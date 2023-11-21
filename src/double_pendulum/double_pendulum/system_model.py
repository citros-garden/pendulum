import numpy as np
from math import sin,cos,radians,sqrt

# double pendulum consists of pendulum number 1 and number 2, pendulums number 2 and number 3 are connected by a spring
# all quantities are in International System of Units

class system_model():
    """
    Double pendulum consists of pendulum number 1 and number 2, pendulums number 2 and number 3 are connected by a spring.
    All quantities are in International System of Units
    """
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
        g=9.81
        a1=w[0]
        a2=w[1]
        v1=w[2]
        v2=w[3]
        delta = a1 - a2
        s_num = -v2**2*sin(delta)/cos(delta)-(self.m1+self.m2)*g*sin(a1)/(self.m2*self.l2*cos(delta))-v1**2*self.l1/self.l2*sin(delta)+g/self.l2*sin(a2)
        s_den = self.l1*(self.m1+self.m2)/(self.m2*self.l1*cos(delta)) - self.l1/self.l2*cos(delta)
        s = s_num/s_den
        return(s)
    def f4(self, w):
        """ v2'=f4 """
        g=9.81
        a1=w[0]
        a2=w[1]
        v1=w[2]
        v2=w[3]
        delta = a1 - a2
        s_num = -v2**2*self.m2*self.l2*sin(delta)/((self.m1+self.m2)*self.l1) - g*sin(a1)/self.l1-v1**2*sin(delta)/cos(delta)+g*sin(a2)/(self.l1*cos(delta))
        s_den = self.m2*self.l2/((self.m1+self.m2)*self.l1)*cos(delta) - self.l2/(self.l1*cos(delta))
        s = s_num/s_den
        return(s)
    def f1(self, w):
        """ a1'=f1 """
        return(w[2])
    def f2(self, w):
        """ a2'=f2 """
        return(w[3])

    def calculate(self, T = 10.0, h = 0.001):
        N=int(T/h+1)
        f=np.array([self.f1,self.f2,self.f3,self.f4])
        eta0=np.array([self.a1_0,self.a2_0,self.v1_0,self.v2_0],dtype=float)
        k1=np.array([0 for i in range(4)],dtype=float)
        k2=np.array([0 for i in range(4)],dtype=float)
        k3=np.array([0 for i in range(4)],dtype=float)
        k4=np.array([0 for i in range(4)],dtype=float)
        eta=np.array([[0 for i in range(4)] for j in range(N)],dtype=float)
        eta[0]=eta0
        for n in range(N-1):
            for i in range(4):
                k1[i]=f[i](eta[n])
                k2[i]=f[i](eta[n]+0.5*h*k1[i])
                k3[i]=f[i](eta[n]+0.5*h*k2[i])
                k4[i]=f[i](eta[n]+h*k3[i])
            eta[n+1]=eta[n]+1/6*h*(k1+2*k2+2*k3+k4)
        x1=self.l1*np.sin(eta[:,0])
        x2=self.l1*np.sin(eta[:,0])+self.l2*np.sin(eta[:,1])
        y1=-self.l1*np.cos(eta[:,0])
        y2=-(self.l1*np.cos(eta[:,0])+self.l2*np.cos(eta[:,1]))
        t=np.array([i*h for i in range(N)])
        return t, x1, x2, y1, y2
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