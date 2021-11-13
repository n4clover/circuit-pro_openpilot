from common.numpy_fast import clip, interp
import numpy as np
import os
from random import randint
from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, \
  create_scc11, create_scc12, create_scc13, create_scc14, \
  create_mdps12, create_lfahda_mfc, create_hda_mfc, create_spas11, create_spas12, create_ems_366, create_eems11, create_ems11
from selfdrive.car.hyundai.scc_smoother import SccSmoother
from selfdrive.car.hyundai.values import Buttons, CAR, FEATURES, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from common.params import Params

from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.road_speed_limiter import road_speed_limiter_get_active

VisualAlert = car.CarControl.HUDControl.VisualAlert
min_set_speed = 30 * CV.KPH_TO_MS
###### SPAS ######
STEER_ANG_MAX = 350         # SPAS Max Angle
#MAX DELTA V limits values
ANGLE_DELTA_BP = [0., 6., 16.]
ANGLE_DELTA_V = [1.0, 0.7, 0.3]    # windup limit
ANGLE_DELTA_VU = [1.3, 0.9, 0.3]   # unwind limit
TQ = 285 # = 1 NM * 100 is unit of measure for wheel.
SPAS_SWITCH = 38 * CV.MPH_TO_MS #MPH
###### SPAS #######

SP_CARS = [CAR.GENESIS, CAR.GENESIS_G70, CAR.GENESIS_G80,
           CAR.GENESIS_EQ900, CAR.GENESIS_EQ900_L, CAR.K9, CAR.GENESIS_G90]

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane, right_lane,
                      left_lane_depart, right_lane_depart):

  sys_warning = (visual_alert in [VisualAlert.steerRequired, VisualAlert.ldw])

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in SP_CARS else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in SP_CARS else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.apply_steer_last = 0
    self.steer_rate_limited = False
    self.lkas11_cnt = 0
    self.scc12_cnt = -1

    self.enabled_cnt = 0
    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    self.turning_signal_timer = 0
    self.longcontrol = CP.openpilotLongitudinalControl
    self.scc_live = not CP.radarOffCan

    self.mad_mode_enabled = Params().get_bool('MadModeEnabled')
    self.cnt = 0
    self.cut_steer = False
    self.emsType = CP.emsType

    if CP.spasEnabled:
      self.last_apply_angle = 0.0
      self.en_spas = 2
      self.mdps11_stat_last = 0
      self.lkas_active = False
      self.spas_active = False
      self.spas_active_last = 0
      self.assist = False
      self.override = False
      
    self.ldws_opt = Params().get_bool('IsLdwsCar')
    self.stock_navi_decel_enabled = Params().get_bool('StockNaviDecelEnabled')

    # gas_factor, brake_factor
    # Adjust it in the range of 0.7 to 1.3
    self.scc_smoother = SccSmoother()
  
  def update(self, enabled, CS, frame, CC, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart, set_speed, lead_visible, controls):

    # Steering Torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque,
                                                CarControllerParams)

    self.steer_rate_limited = new_steer != apply_steer

    # SPAS limit angle extremes for safety
    if CS.spas_enabled:
      apply_angle = actuators.steeringAngleDeg
      if abs(apply_angle - CS.out.steeringAngleDeg) > 8: # Rate limit for when steering angle is far from apply_angle - JPR
        rate_limit = 8
        apply_angle = clip(actuators.steeringAngleDeg, CS.out.steeringAngleDeg - rate_limit, CS.out.steeringAngleDeg + rate_limit)
      else:
        if self.last_apply_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_apply_angle):
          rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        else:
          rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
        apply_angle = clip(actuators.steeringAngleDeg, self.last_apply_angle - rate_limit, self.last_apply_angle + rate_limit)    
      self.last_apply_angle = apply_angle
        
    if not self.cut_steer:
      spas_active = CS.spas_enabled and enabled and CS.out.vEgo < SPAS_SWITCH
      if CS.spas_enabled:
        lkas_active = enabled and not CS.out.steerWarning and abs(CS.out.steeringAngleDeg) < CS.CP.maxSteeringAngleDeg and not CS.mdps11_stat == 5
      else:
        lkas_active = enabled and not CS.out.steerWarning and abs(CS.out.steeringAngleDeg) < CS.CP.maxSteeringAngleDeg
    else:
      if CS.spas_enabled:
        spas_active = False
      lkas_active = False

    if CS.spas_enabled:
      if abs(CS.out.steeringWheelTorque) > TQ and spas_active and not lkas_active:
        self.override = True
        print("OVERRIDE")
      else:
        self.override = False

    # Disable steering while turning blinker on and speed below 60 kph
    if CS.out.leftBlinker or CS.out.rightBlinker:
      self.turning_signal_timer = 0.5 / DT_CTRL  # Disable for 1.0 Seconds after blinker turned off
      
    if self.turning_indicator_alert and enabled: # set and clear by interface
      self.cut_steer = True
      
    if abs(CS.out.steeringWheelTorque) < 30:
        self.cut_steer = False

    if not lkas_active:
      apply_steer = 0

    self.lkas_active = lkas_active
    self.spas_active = spas_active

    if self.turning_signal_timer > 0:
      self.turning_signal_timer -= 1  

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = \
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph else 60
    if clu11_speed > enabled_speed or not lkas_active:
      enabled_speed = clu11_speed

    if not (min_set_speed < set_speed < 255 * CV.KPH_TO_MS):
      set_speed = min_set_speed
    set_speed *= CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

    if frame == 0:  # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"]

      # TODO: fix this
      # self.prev_scc_cnt = CS.scc11["AliveCounterACC"]
      # self.scc_update_frame = frame

    # check if SCC is alive
    # if frame % 7 == 0:
    # if CS.scc11["AliveCounterACC"] == self.prev_scc_cnt:
    # if frame - self.scc_update_frame > 20 and self.scc_live:
    # self.scc_live = False
    # else:
    # self.scc_live = True
    # self.prev_scc_cnt = CS.scc11["AliveCounterACC"]
    # self.scc_update_frame = frame

    self.prev_scc_cnt = CS.scc11["AliveCounterACC"]

    self.lkas11_cnt = (self.lkas11_cnt + 1) % 0x10

    can_sends = []
    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, 0, self.ldws_opt))

    if CS.mdps_bus or CS.scc_bus == 1:  # send lkas11 bus 1 if mdps or scc is on bus 1
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                     CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
                                     left_lane_warning, right_lane_warning, 1, self.ldws_opt))

    if frame % 2 and CS.mdps_bus: # send clu11 to mdps if it is not on bus 0
      can_sends.append(create_clu11(self.packer, frame // 2 % 0x10, CS.mdps_bus, CS.clu11, Buttons.NONE, enabled_speed))

    if pcm_cancel_cmd and (self.longcontrol and not self.mad_mode_enabled) or not enabled and self.enabled_cnt >= 1: #Make SCC cancle when op disengage -JPR
      can_sends.append(create_clu11(self.packer, frame % 0x10, CS.scc_bus, CS.clu11, Buttons.CANCEL, clu11_speed))
      self.enabled_cnt += 1
    else:
      can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    if enabled: # Enable count so we don't spam cancel to SCC - JPR
      self.enabled_cnt = 1
    if self.enabled_cnt == 3:
      self.enabled_cnt = 0

    # fix auto resume - by neokii
    if CS.out.cruiseState.standstill and not CS.out.gasPressed:

      if self.last_lead_distance == 0:
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
        self.resume_wait_timer = 0

      # scc smoother
      elif self.scc_smoother.is_active(frame):
        pass

      elif self.resume_wait_timer > 0:
        self.resume_wait_timer -= 1

      elif abs(CS.lead_distance - self.last_lead_distance) > 0.1:
        can_sends.append(create_clu11(self.packer, self.resume_cnt, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.resume_cnt += 1

        if self.resume_cnt >= randint(6, 8):
          self.resume_cnt = 0
          self.resume_wait_timer = randint(30, 36)

    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0

    # scc smoother
    self.scc_smoother.update(enabled, can_sends, self.packer, CC, CS, frame, controls)

    # send scc to car if longcontrol enabled and SCC not on bus 0 or ont live
    if self.longcontrol and CS.cruiseState_enabled and (CS.scc_bus or not self.scc_live):

      if frame % 2 == 0:
        
        stopping = controls.LoC.long_control_state == LongCtrlState.stopping
        apply_accel = self.scc_smoother.get_apply_accel(CS, controls.sm, actuators.accel, stopping)
        apply_accel = clip(apply_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

        controls.apply_accel = apply_accel
        aReqValue = CS.scc12["aReqValue"]
        controls.aReqValue = aReqValue

        if aReqValue < controls.aReqValueMin:
          controls.aReqValueMin = controls.aReqValue

        if aReqValue > controls.aReqValueMax:
          controls.aReqValueMax = controls.aReqValue

        if self.stock_navi_decel_enabled:
          controls.sccStockCamAct = CS.scc11["Navi_SCC_Camera_Act"]
          controls.sccStockCamStatus = CS.scc11["Navi_SCC_Camera_Status"]
          apply_accel, stock_cam = self.scc_smoother.get_stock_cam_accel(apply_accel, aReqValue, CS.scc11)
        else:
          controls.sccStockCamAct = 0
          controls.sccStockCamStatus = 0
          stock_cam = False

        if self.scc12_cnt < 0:
          self.scc12_cnt = CS.scc12["CR_VSM_Alive"] if not CS.no_radar else 0

        self.scc12_cnt += 1
        self.scc12_cnt %= 0xF

        can_sends.append(create_scc12(self.packer, apply_accel, enabled, self.scc12_cnt, self.scc_live, CS.scc12,
                                      CS.out.gasPressed, CS.out.brakePressed, CS.out.cruiseState.standstill,
                                      self.car_fingerprint))

        can_sends.append(create_scc11(self.packer, frame, enabled, set_speed, lead_visible, self.scc_live, CS.scc11,
                                      self.scc_smoother.active_cam, stock_cam))

        if frame % 20 == 0 and CS.has_scc13:
          can_sends.append(create_scc13(self.packer, CS.scc13))
          
        if CS.has_scc14:
          acc_standstill = stopping if CS.out.vEgo < 2. else False

          lead = self.scc_smoother.get_lead(controls.sm)

          if lead is not None:
            d = lead.dRel
            obj_gap = 1 if d < 25 else 2 if d < 40 else 3 if d < 60 else 4 if d < 80 else 5
          else:
            obj_gap = 0

          can_sends.append(create_scc14(self.packer, enabled, CS.out.vEgo, acc_standstill, apply_accel, CS.out.gasPressed,
                                        obj_gap, CS.scc14))
    else:
      self.scc12_cnt = -1

    # 20 Hz LFA MFA message
    if frame % 5 == 0:
      activated_hda = road_speed_limiter_get_active()
      # activated_hda: 0 - off, 1 - main road, 2 - highway
      if self.car_fingerprint in FEATURES["send_lfa_mfa"]:
        can_sends.append(create_lfahda_mfc(self.packer, enabled, activated_hda))
      elif CS.mdps_bus == 0:
        state = 2 if self.car_fingerprint in FEATURES["send_hda_state_2"] else 1
        can_sends.append(create_hda_mfc(self.packer, activated_hda, state))
    
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
        if spas_active: # Spoof Speed on mdps11_stat 4 and 5 JPR
          if CS.mdps11_stat == 4 or CS.mdps11_stat == 5 or CS.mdps11_stat == 3: 
            spas_active_stat = True
          else:
            spas_active_stat = False
        if self.emsType == 1:
          can_sends.append(create_ems_366(self.packer, CS.ems_366, spas_active_stat))
          if Params().get_bool('SPASDebug'):
            print("EMS_366")
        elif self.emsType == 2:
          can_sends.append(create_ems11(self.packer, CS.ems11, spas_active_stat))
          if Params().get_bool('SPASDebug'):
            print("EMS_11")
        elif self.emsType == 3:
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
          print("lkas_active:", lkas_active)
          print("driver torque:", CS.out.steeringWheelTorque)
          if self.emsType == 0:
            print("Please add a car parameter called ret.emsType = (your EMS type) in interface.py : EMS_366 = 1 : EMS_11 = 2 : E_EMS11 = 3")
      # SPAS12 20Hz
      if (frame % 5) == 0:
        can_sends.append(create_spas12(CS.mdps_bus))
      self.spas_active_last = spas_active
    return can_sends
