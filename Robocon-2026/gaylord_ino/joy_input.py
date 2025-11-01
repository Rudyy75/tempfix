import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interfaces.msg import Rpm 
from robot_interfaces.msg import Pwm
from std_msgs.msg import Float32

class joy_input(Node):
    def __init__(self):
        super().__init__("joy_input")
        self.publisher = self.create_publisher(Pwm,"/pwm",10) #puts a final pid value into the microcontroller
        self.subscription1 = self.create_subscription(Joy, "/joy", self.subscriber_callback,10)
        self.subscription2 = self.create_subscription(Rpm,"/rpm", self.encoder_callback,10)
        self.timer=self.create_timer(0.1,self.my_publisher)
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.sum=0.0
        self.prev_error = 0.0 #idk if i wanna use this
    
    def encoder_callback(self, msg):
        msg= Rpm()
        self.current_speed =msg.rpm
        

 
    def subscriber_callback(self, msg):
        self.target_speed = 50*msg.axes[1]


    def my_publisher(self):
        current_pwm=Pwm()
        kp=0.1
        ki=0.00000001
        kd=0.00001
        dt=0.1
                
        error=self.target_speed-self.current_speed
        self.sum += error*dt
        pwm = kp*error + ki*self.sum + kd*(error-self.prev_error)/dt
        self.prev_error = error
        current_pwm.pwm = float(pwm)
        
        self.publisher.publish(current_pwm ) 

def main(args=None):
    rclpy.init(args=args)
    node = joy_input()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()



