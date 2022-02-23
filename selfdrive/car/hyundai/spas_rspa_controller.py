# This is the work of JPR
from cereal import car
from common.params import Params
from selfdrive.car.hyundai.hyundaican import create_spas11, create_spas12, create_ems_366, create_eems11, create_ems11
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker

###### SPAS ###### - JPR
STEER_ANG_MAX = 450 # SPAS Max Angle
ANGLE_DELTA_BP = [0., 10., 20.]
ANGLE_DELTA_V = [1.19, 1.14, 1.09]    # windup limit
ANGLE_DELTA_VU = [1.29, 1.19, 1.14]   # unwind limit
TQ = 290 # = TQ / 100 = NM is unit of measure for wheel.
SPAS_SWITCH = 30 * CV.MPH_TO_MS # MPH - lowered Bc of model and overlearn steerRatio
STEER_MAX_OFFSET = 105 # How far from MAX LKAS torque to engage Dynamic SPAS when under 60mph.
###### SPAS #######

class SpasRspaController:
  def __init__(self):
    self.last_apply_angle = 0.0
    self.en_spas = 2
    self.mdps11_stat_last = 0
    self.lkas_active = False
    self.spas_active = False
    self.spas_active_last = False
    self.dynamicSpas = Params().get_bool('DynamicSpas')
    self.ratelimit = 2.3 # Starting point - JPR
    self.rate = 0
    self.lastSteeringAngleDeg = 0
    self.cut_timer = 0

  def update(self, c, enabled, CS, actuators, frame, maxTQ, packer, car_fingerprint, emsType, apply_steer, turnsignalcut):
    self.packer = packer
    self.car_fingerprint = car_fingerprint

    # Keep Track of Steering wheel rate - JPR
    self.rate = abs(CS.out.steeringAngleDeg - self.lastSteeringAngleDeg)
    # SPAS
    if CS.spas_enabled and enabled:
      apply_angle = clip(actuators.steeringAngleDeg, -1*(STEER_ANG_MAX), STEER_ANG_MAX)
      apply_diff = abs(apply_angle - CS.out.steeringAngleDeg)
      if apply_diff > 1.75 and c.active: # Rate limit for when steering angle is not apply_angle - JPR
        self.ratelimit = self.ratelimit + 0.03 # Increase each cycle - JPR
        rate_limit = max(self.ratelimit, 10) # Make sure not to go past +-10 on rate - JPR
        #print("apply_diff is greater than 1.5 : rate limit :", rate_limit)
        apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - rate_limit, CS.out.steeringAngleDeg + rate_limit)
      elif c.active:
        self.ratelimit = 2.3 # Reset it back - JPR
        if self.last_apply_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_apply_angle):
          rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        else:
          rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
        apply_angle = clip(apply_angle, self.last_apply_angle - rate_limit, self.last_apply_angle + rate_limit)

      self.last_apply_angle = apply_angle
    spas_active = CS.spas_enabled and c.active and CS.out.vEgo < 26.82 and (CS.out.vEgo < SPAS_SWITCH or apply_diff > 3.2 and self.dynamicSpas and not CS.out.steeringPressed or abs(apply_angle) > 3. and self.spas_active or maxTQ - STEER_MAX_OFFSET < apply_steer and self.dynamicSpas)

    if CS.spas_enabled and enabled:
      if CS.out.steeringPressed:
        self.cut_timer = 0
      if CS.out.steeringPressed or self.cut_timer < 85 and self.rate < 5:# Keep SPAS cut for 50 cycles after steering pressed to prevent unintentional fighting. - JPR
        spas_active = False
        self.cut_timer += 1
    
      if turnsignalcut:
        spas_active = False

    can_sends = []

   ############### SPAS STATES ############## JPR
# State 1 : Start
# State 2 : New Request
# State 3 : Ready to Assist(Steer)
# State 4 : Hand Shake between OpenPilot and MDPS ECU
# State 5 : Assisting (Steering)
# State 6 : Failed to Assist (Steer)
# State 7 : Cancel
# State 8 : Failed to get ready to Assist (Steer)
# ---------------------------------------------------
    if CS.spas_enabled:
      if CS.mdps_bus:
        spas_active_stat = False
        if spas_active: # Spoof Speed on mdps11_stat 3, 4 and 5 JPR
          if CS.mdps11_stat == 4 or CS.mdps11_stat == 5 or CS.mdps11_stat == 3:
            spas_active_stat = True
          else:
            spas_active_stat = False
        if emsType == 1:
          can_sends.append(create_ems_366(self.packer, CS.ems_366, spas_active_stat))
          if Params().get_bool('SPASDebug'):
            print("EMS_366")
        elif emsType == 2:
          can_sends.append(create_ems11(self.packer, CS.ems11, spas_active_stat))
          if Params().get_bool('SPASDebug'):
            print("EMS_11")
        elif emsType == 3:
          can_sends.append(create_eems11(self.packer, CS.eems11, spas_active_stat))
          if Params().get_bool('SPASDebug'):
            print("E_EMS11")

      if (frame % 2) == 0:
        if CS.mdps11_stat == 7:
            self.en_spas = 7

        if CS.mdps11_stat == 7 and self.mdps11_stat_last == 7:
          self.en_spas = 3
          if CS.mdps11_stat == 3:
            self.en_spas = 2

        if CS.mdps11_stat == 2 and spas_active:
          self.en_spas = 3 # Switch to State 3, and get Ready to Assist(Steer). JPR

        if CS.mdps11_stat == 3 and spas_active:
          self.en_spas = 4

        if CS.mdps11_stat == 4 and spas_active:
          self.en_spas = 5

        if CS.mdps11_stat == 5 and not spas_active:
          self.en_spas = 7

        if CS.mdps11_stat == 6: # Failed to Assist and Steer, Set state back to 2 for a new request. JPR
          self.en_spas = 2

        if CS.mdps11_stat == 8: #MDPS ECU Fails to get into state 3 and ready for state 5. JPR
          self.en_spas = 2

        if not spas_active:
          apply_angle = CS.mdps11_strang

        self.mdps11_stat_last = CS.mdps11_stat
        can_sends.append(create_spas11(self.packer, self.car_fingerprint, (frame // 2), self.en_spas, apply_angle, CS.mdps_bus))
        if Params().get_bool('SPASDebug'):
          print("MDPS SPAS State: ", CS.mdps11_stat) # SPAS STATE DEBUG
          print("OP SPAS State: ", self.en_spas) # OpenPilot Ask MDPS to switch to state.
          print("spas_active:", spas_active)
          print("apply angle:", apply_angle)
          print("driver torque:", CS.out.steeringWheelTorque)
        if emsType == 0:
          print("Please add a car parameter called ret.emsType = (your EMS type) in interface.py : EMS_366 = 1 : EMS_11 = 2 : E_EMS11 = 3")

      # SPAS12 20Hz
      if (frame % 5) == 0:
        can_sends.append(create_spas12(CS.mdps_bus))

      self.spas_active_last = self.spas_active
      self.spas_active = spas_active
      self.lastSteeringAngleDeg = CS.out.steeringAngleDeg
      return can_sends

    
      





