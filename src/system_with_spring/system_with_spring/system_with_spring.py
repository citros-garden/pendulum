import rclpy
from rclpy.node import Node

from system_with_spring_interfaces.msg import SpringSystem, PendulumCoord, SpringCoord

from .system_model import system_model

class system_with_spring(Node):
    def __init__(self):
        super().__init__('pendulum')

        self.publish = self.create_publisher(SpringSystem , 'coordinates', 10)

        self.declare_parameter('l1', 0.08) # pendulum lengths, m
        self.declare_parameter('l2', 0.2)
        self.declare_parameter('l3', 0.32)
        self.declare_parameter('lk', 0.14) # spring attachment point, m
        self.declare_parameter('m1', 5.0) # masses, kg
        self.declare_parameter('m2', 3.0)
        self.declare_parameter('m3', 1.0)
        self.declare_parameter('a1_0', 30.0) # initial deviation of pendulums from the vertical, degrees
        self.declare_parameter('a2_0', 10.0)
        self.declare_parameter('a3_0', -30.0)
        self.declare_parameter('v1_0', 0.0) # initial radial velocities, degrees/s
        self.declare_parameter('v2_0', 0.0)
        self.declare_parameter('v3_0', 0.0)
        self.declare_parameter('x0', 0.1) # horizontal distance between pendulums, m
        self.declare_parameter('k', 100.0)   # spring constant, kg/s^2
        self.declare_parameter('l0', 0.05) # unstretched spring length, m

        self.declare_parameter('T', 10.0) # time of calculation, s
        self.declare_parameter('h', 0.01) # step, s

        l1 = self.get_parameter('l1').get_parameter_value().double_value
        l2 = self.get_parameter('l2').get_parameter_value().double_value
        l3 = self.get_parameter('l3').get_parameter_value().double_value
        lk = self.get_parameter('lk').get_parameter_value().double_value
        m1 = self.get_parameter('m1').get_parameter_value().double_value
        m2 = self.get_parameter('m2').get_parameter_value().double_value
        m3 = self.get_parameter('m3').get_parameter_value().double_value
        a1_0 = self.get_parameter('a1_0').get_parameter_value().double_value
        a2_0 = self.get_parameter('a2_0').get_parameter_value().double_value
        a3_0 = self.get_parameter('a3_0').get_parameter_value().double_value
        v1_0 = self.get_parameter('v1_0').get_parameter_value().double_value
        v2_0 = self.get_parameter('v2_0').get_parameter_value().double_value
        v3_0 = self.get_parameter('v3_0').get_parameter_value().double_value
        x0 = self.get_parameter('x0').get_parameter_value().double_value
        k = self.get_parameter('k').get_parameter_value().double_value
        l0 = self.get_parameter('l0').get_parameter_value().double_value

        T = self.get_parameter('T').get_parameter_value().double_value
        h = self.get_parameter('h').get_parameter_value().double_value

        pendulum = system_model(l1 = l1, l2 = l2, l3 = l3, lk=lk , m1=m1, m2=m2, m3=m3,
                 a1_0=a1_0, a2_0=a2_0, a3_0 = a3_0, v1_0 = v1_0, v2_0 = v2_0, v3_0 = v3_0, 
                 x0 = x0, k=k, l0 = l0)
        
        self.t, self.x1, self.x2, self.x3, self.xk_0, self.xk_1, self.y1, self.y2, self.y3, self.yk_0, self.yk_1 = pendulum.calculate(T = T, h = h)
        self.data_length = len(self.t)

        self.publish_msg = SpringSystem()
        self.p1_msg = PendulumCoord()
        self.p2_msg = PendulumCoord()
        self.p3_msg = PendulumCoord()
        self.spr_msg = SpringCoord()
        
        #Defining inputs
        self.declare_parameter('publish_freq', 10.0)   

        # Defining encounter for publisher
        self.i = 0

        timer_period = 1/self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        self.p1_msg.x = self.x1[self.i]
        self.p2_msg.x = self.x2[self.i]
        self.p3_msg.x = self.x3[self.i]
        self.spr_msg.x0 = self.xk_0[self.i]
        self.spr_msg.x1 = self.xk_1[self.i]

        self.p1_msg.y = self.y1[self.i]
        self.p2_msg.y = self.y2[self.i]
        self.p3_msg.y = self.y3[self.i]
        self.spr_msg.y0 = self.yk_0[self.i]
        self.spr_msg.y1 = self.yk_1[self.i]

        self.publish_msg.t = self.t[self.i]
        self.publish_msg.p1 = self.p1_msg
        self.publish_msg.p2 = self.p2_msg
        self.publish_msg.p3 = self.p3_msg
        self.publish_msg.spr = self.spr_msg

        self.publish.publish(self.publish_msg)

        self.get_logger().info(f"Publishing via self.pos_GT_pub = {self.publish_msg.t}, {self.publish_msg.p1}, {self.publish_msg.p2}, {self.publish_msg.p3}, {self.publish_msg.spr}")

        self.i += 1
        if self.i==self.data_length:
            self.get_logger().info('All data published successfully')
            exit()

        
def main(args=None):
    rclpy.init(args=args)
    pendulum = system_with_spring()
    rclpy.spin(pendulum)
    pendulum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()