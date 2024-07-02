import numpy as np
from puzzlebot_assembly.canvas import Canvas
from puzzlebot_assembly.logging import Logger
from puzzlebot_assembly.behavior_lib import BehaviorLib

class RobotParam:
    def __init__(self, L=5e-2,
            anchor_base_L=8e-3,
            anchor_L=1e-2):
        self.L = L
        self.anchor_base_L = anchor_base_L
        self.anchor_L = anchor_L

class Robots:
    def __init__(self, N, controller, robot_param=None, eth=1e-3, pilot_ids=[]):
        self.N = N
        self.ctl = controller
        self.x = np.zeros(3*N)
        self.u = np.zeros(2*N)
        self.prev_u = np.zeros(2*N)
        self.log = Logger()
        self.canvas = Canvas(N)
        self.time = {}
        self.pilot_ids = pilot_ids
        self.robot_config = robot_param if robot_param else RobotParam()
        self.behavs = BehaviorLib(N, controller, eth=eth, 
                                robot_param=self.robot_config)

    def setup(self, start, dt=0.1, tmax=15):
        assert(self.N == start.shape[1])
        
        self.time = {'t': 0, 'dt': dt, 'tmax': tmax}
        #  self.behavs.add_bhav(self.behavs.align_cp_pool)
        #  self.behavs.add_bhav(self.behavs.wiggle)
        self.behavs.add_bhav(self.behavs.random)
        #  self.behavs.add_bhav(self.behavs.anchor_couple)
        #  self.behavs.add_bhav(self.behavs.align_anchor_pool)
        #  self.behavs.add_bhav(self.behavs.go_to_goal)
        #  self.behavs.add_bhav(self.behavs.go_du)
        self.behavs.add_bhav(self.behavs.nothing)

        self.x = start.T.flatten()

    def start(self):
        # this function is replaced in the simulation.py for Bullet
        print("System started.")
        while self.time['t'] < self.time['tmax']:
            is_done = self.step(self.x, self.prev_u, self.time['t'])
            x = self.ctl.fk(self.x, self.u)
            #  x = self.ctl.fk_rk4(self.x, self.u)
            #  x = self.ctl.fk_exact(self.x, self.u)
            self.log.append_xu(self.x, self.u)
            self.x = x
            self.time['t'] += self.time['dt']
            if is_done: 
                print("Simulation ended.")
                break

    def step(self, x, u, t):
        """
        x: 3N vector, 
        """
        N = self.N
        body_len = self.robot_config.L
        curr_bhav = self.behavs.current()
        if not curr_bhav: return True
        if curr_bhav == self.behavs.align_cp:
            cp = {(0, 1): np.array([[body_len/2, body_len/2, 0],
                    [-body_len/2, body_len/2, 0]]).T}
            u = curr_bhav(x, u, cp)
        elif curr_bhav == self.behavs.go_du:
            curr_key = list(self.behavs.anchor_cps.keys())[0]
            prev_cp = {curr_key: self.behavs.anchor_cps[curr_key]['align_cp']}
            gdu = np.array([[-0.05, 0.5] for i in range(N)]).flatten()
            u = curr_bhav(x, u, gdu, prev_cp=prev_cp)
        elif curr_bhav == self.behavs.wiggle:
            u = curr_bhav(x, u, t=t, vbias=0.005, rid=[0])
        else:
            u = curr_bhav(x, u)
        if u is None:
            self.behavs.bhav_id += 1
            self.u[:] = 0
            return False
        self.prev_u[:] = self.u[:]
        self.u = u
        return False

    def generate_anim(self):
        self.canvas.animation(self.log)
