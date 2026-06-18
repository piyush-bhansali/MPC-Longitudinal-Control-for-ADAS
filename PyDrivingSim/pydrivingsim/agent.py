import time
from math import cos, sin

import rclpy
from rclpy.node import Node
from adas_msgs.msg import ScenarioMsg, ManoeuvreMsg

from pydrivingsim import World, Vehicle, TrafficLight, TrafficCone, SuggestedSpeedSignal, Coin


class _AgentNode(Node):
    def __init__(self):
        super().__init__('simulator_agent')
        self.scenario_pub = self.create_publisher(ScenarioMsg, '/scenario', 10)
        self.create_subscription(ManoeuvreMsg, '/manoeuvre', self._manoeuvre_cb, 10)
        self.last_manoeuvre = None

    def _manoeuvre_cb(self, msg):
        self.last_manoeuvre = msg


class Agent():
    __metadata = {"dt": 0.05}

    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle

        assert self.__metadata["dt"] >= World().get_dt()
        self.sim_call_freq = self.__metadata["dt"] / World().get_dt()
        self.num_of_step = 0
        self.cycle_number = 0
        self.requested_cruising_speed = 20
        self.action = (0, 0)

        self.ALgtFild    = 0
        self.YawRateFild = 0
        self.SteerWhlAg  = 0

        if not rclpy.ok():
            rclpy.init()
        self._node = _AgentNode()

    def compute(self):
        self.num_of_step += 1
        self.__filtering(self.vehicle)
        if self.num_of_step >= self.sim_call_freq:
            self.__compute(self.vehicle)
            self.num_of_step = 0
            self.__clear_filter()

    def __filtering(self, v: Vehicle):
        self.ALgtFild    += v.dX[3] - v.state[5] * v.state[4]
        self.YawRateFild += v.state[5]
        self.SteerWhlAg  += v.state[10]

    def __clear_filter(self):
        self.ALgtFild    = 0
        self.YawRateFild = 0
        self.SteerWhlAg  = 0

    def __compute(self, v: Vehicle):
        if self._node.last_manoeuvre and self._node.last_manoeuvre.status == 1:
            self.terminate()
            return

        self.cycle_number += 1

        msg = ScenarioMsg()
        msg.cycle_number = self.cycle_number
        msg.id           = 0
        msg.status       = 0
        msg.ecu_up_time  = World().time

        msg.vehicle_len              = float(v.vehicle.vehicle.L)
        msg.vehicle_width            = float(v.vehicle.vehicle.Wf)
        msg.lane_heading             = float(-v.state[2])
        msg.v_lgt_fild               = float(v.state[3])
        msg.a_lgt_fild               = float(self.ALgtFild    / self.num_of_step)
        msg.yaw_rate_fild            = float(self.YawRateFild / self.num_of_step)
        msg.steer_whl_ag             = float(self.SteerWhlAg  / self.num_of_step)
        msg.requested_cruising_speed = float(self.requested_cruising_speed)

        road_width = 4
        if abs(v.state[1] - road_width / 2) <= road_width:
            msg.lane_width      = float(road_width)
            msg.lat_offs_line_r = (msg.lane_width - road_width / 2) - v.state[1]
            msg.lat_offs_line_l = -road_width / 2 - v.state[1]

        trafficlight     = None
        trafficlightDist = float('inf')
        speedlimitId     = 0
        objId            = 0

        for obj in World().obj_list:
            if type(obj) is TrafficLight:
                dist = obj.pos[0] - v.state[0]
                if dist > -1.0 and dist < trafficlightDist:
                    trafficlightDist = dist
                    trafficlight = obj

            if type(obj) is TrafficCone:
                delta_x = obj.pos[0] - float(v.state[0])
                delta_y = obj.pos[1] - float(v.state[1])
                msg.obj_id[objId]    = 1
                msg.obj_x[objId]     = delta_x * cos(float(v.state[2])) + delta_y * sin(float(v.state[2]))
                msg.obj_y[objId]     = -delta_x * sin(float(v.state[2])) + delta_y * cos(float(v.state[2]))
                msg.obj_vel[objId]   = 0.0
                msg.obj_len[objId]   = float(obj.size)
                msg.obj_width[objId] = float(obj.size)
                objId += 1

            if type(obj) is Coin:
                delta_x = obj.pos[0] - float(v.state[0])
                delta_y = obj.pos[1] - float(v.state[1])
                msg.obj_id[objId]    = 2
                msg.obj_x[objId]     = delta_x * cos(float(v.state[2])) + delta_y * sin(float(v.state[2]))
                msg.obj_y[objId]     = -delta_x * sin(float(v.state[2])) + delta_y * cos(float(v.state[2]))
                msg.obj_vel[objId]   = 0.0
                msg.obj_len[objId]   = float(obj.size)
                msg.obj_width[objId] = float(obj.size)
                objId += 1

            if type(obj) is SuggestedSpeedSignal:
                msg.adasis_speed_limit_values[speedlimitId] = int(obj.vel)
                msg.adasis_speed_limit_dist[speedlimitId]   = obj.pos[0] - v.state[0]
                speedlimitId += 1

        msg.nr_objs               = objId
        msg.adasis_speed_limit_nr = speedlimitId

        msg.nr_trf_lights = 0
        if trafficlight:
            state = trafficlight.state
            t1 = float(trafficlight.time_phases[state]) - trafficlight.time_past_switch
            t2 = t1 + float(trafficlight.time_phases[divmod(state + 1, 3)[1]])
            t3 = t2 + float(trafficlight.time_phases[divmod(state + 2, 3)[1]])
            msg.nr_trf_lights                   = 1
            msg.trf_light_dist                  = float(trafficlight.pos[0]) - float(v.state[0])
            msg.trf_light_curr_state            = state + 1  # 1=Green 2=Yellow 3=Red
            msg.trf_light_first_time_to_change  = t1
            msg.trf_light_first_next_state      = divmod(state + 1, 3)[1] + 1
            msg.trf_light_second_time_to_change = t2
            msg.trf_light_second_next_state     = divmod(state + 2, 3)[1] + 1
            msg.trf_light_third_time_to_change  = t3

        self._node.last_manoeuvre = None
        self._node.scenario_pub.publish(msg)

        deadline = time.time() + 0.5
        while self._node.last_manoeuvre is None and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.01)

        if self._node.last_manoeuvre:
            m = self._node.last_manoeuvre
            self.action = (m.requested_acc, m.requested_steer_whl_ag)

    def terminate(self):
        World().loop = 0

    def get_action(self):
        return self.action
