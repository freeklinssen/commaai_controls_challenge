from . import BaseController
import numpy as np
import math

class Controller(BaseController):
  """
  Proportional-Integral-Derivative (PID) controller
  """
  def __init__(self,):
    self.counter = 0
    self.p = 0.28
    self.i = 0.08
    self.d = 0.09
    
    self.error_integral = 0
    self.prev_error = 0
    
    self.steer_factor = 13 # lat accel to steer command factor
    self.minimum_v = 20 
        

  def update(self, target_lataccel, current_lataccel, state, future_plan):
      ## state hass roll_lataccel, v_eg, a_ego
      
      # reset error history before start:
      self.counter += 1
      if self.counter == 81:
        self.error_integral = 0
        self.prev_error = 0
      
      # because  the response time is slow:
      if len(future_plan.lataccel) >= 5:
        target_lataccel = np.average([target_lataccel] + future_plan.lataccel[0:5], weights = [4, 3, 2, 2, 2, 1]) 
      
      # normal pid
      error = target_lataccel - current_lataccel
      self.error_integral += error
      error_difference = error - self.prev_error
      self.prev_error = error 
      
                           
      # scale p, i, and p down for high accelerations
      p = max(0.12, self.p+0.05 - 0.05*(np.exp(abs(state.a_ego))))
        
      i= max(0.07, self.i+0.005 -0.005*np.exp(abs(target_lataccel)))
        
      d = min(-0.02, -(self.d+0.005 - 0.005*(np.exp(abs(state.a_ego)))))
      
      
      pid_factor = min(1, 1 - (abs(target_lataccel) - 1) * 0.23)
      
      # pid input   
      u_pid = (p * error + i * self.error_integral + d * error_difference) * pid_factor
      
      
      # estimate steer command with target target lateral acceleration  
      steer_lataccel_target = (target_lataccel - state.roll_lataccel)
      steer_command = steer_lataccel_target * (self.steer_factor / max(self.minimum_v, state.v_ego))
      
      # tanh to dampen extremes
      steer_command = 2 * np.tanh(steer_command/2)
      
      return u_pid + 0.80*steer_command