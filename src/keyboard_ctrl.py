import rospy
import numpy as np
import time
from std_msgs.msg import Float32MultiArray

class KeyboardCtrl:
    def __init__(self, N, is_pwm=True):
        self.N = N
        rospy.init_node('keyboard_control', anonymous=True,
                    log_level=rospy.DEBUG)
        self.pub = rospy.Publisher('vel_array', 
                            Float32MultiArray, queue_size=1)
        self.is_pwm = is_pwm
        self.forward_count = 0

    def back_and_forth(self):
        N = self.N
        curr_time = round(time.time())
        vel = np.zeros([2, N])

        if self.forward_count < 40:
            vel[:, :] = -300
        else:
            vel[:, :] = 300
        self.forward_count += 1
        if self.forward_count > 60: self.forward_count = 0
        return vel

    def keyboard_input_each(self, pilot_ids=[]):
        N = self.N
        vel = np.zeros([2, N])
        txt = ''
        if self.is_pwm:
            txt = input("Input PWM values in format [r0 left, r0 right, r1 left, ...]: ")
        else:
            txt = input("Input cmd_vel in format [r0 left, r0 right, r1 left, ...]: ")
            print("not tested yet for non pwm input")
        v_list = []
        try:
            #  print("input:", txt)
            txt = txt.split(',')
            #  if type(txt) is not tuple:
                #  txt = [txt]
            for tx in txt:
                if self.is_pwm:
                    v_list.append(int(tx))
                else:
                    v_list.append(float(tx))
        except Exception as e:
            rospy.logerr(e)
            self.vel = vel
            return vel

        if len(v_list) == 0:
            self.vel = vel
            return vel

        if len(v_list) == 1:
            self.vel = vel
            vel[:, :] =  v_list[0]
            return vel
        elif len(v_list) != 2*N and len(v_list) > 1:
            print('Number of input velocity != 2*N')
            self.vel = vel
            return vel

        v_list = np.array(v_list)
        vel = v_list.reshape([N, 2]).T
        #  vel[0, :] = v_list[0:2*N:2]
        #  vel[1, :] = v_list[1:2*N:2]
        vel[:, pilot_ids] = np.flipud(vel[:, pilot_ids])

        return vel
    
    def start(self):
        rate = rospy.Rate(20)
        vel = self.keyboard_input_each()
        #  vel = self.back_and_forth()
        pub_vel = Float32MultiArray()

        while not rospy.is_shutdown():
            vel = self.keyboard_input_each()
            #  vel = self.back_and_forth()
            pub_vel.data = vel.T.flatten().tolist()
            self.pub.publish(pub_vel)
            rate.sleep()
