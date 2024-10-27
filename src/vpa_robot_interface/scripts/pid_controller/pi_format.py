#!/usr/bin/python3

class PIController:

    """ A class to implement a PI (Proportional-Intergral) controller """

    def __init__(self, kp, ki) -> None:
        # 控制器参数
        self.kp = kp
        self.ki = ki
        self.frequency = 20
        self.dt = 1 / self.frequency

        # 状态变量
        self.last_error = 0
        self.last_output     = 0

    def pi_control(self, ref, sig) -> float:
        if ref == 0:
            self.reset()
            return 0
        
        # current err
        err = ref - sig

        delta_err = err - self.last_error
        
        delta_output = self.kp * delta_err + self.ki * self.dt * err

        output = self.last_output + delta_output

        self.last_error = err
        self.last_output = output

        return output
    
    def update_controller_param(self,kp,ki) -> None:
        self.kp = kp
        self.ki = ki
        
    def reset(self):        
        self.last_error = 0
        self.last_output = 0       