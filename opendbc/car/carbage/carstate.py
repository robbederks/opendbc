import copy
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, structs, CanSignalRateCalculator
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.carbage.values import DBC, STEER_THRESHOLD

from opendbc.car.toyota.carstate import TEMP_STEER_FAULTS, PERM_STEER_FAULTS

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.accurate_steer_angle_seen = False
    self.rpm = FirstOrderFilter(0, 1.0, DT_CTRL)
    self.angle_rate_calculator = CanSignalRateCalculator(50)

    self.steer_angle = None
    self.steer_fraction = None
    self.steer_rate = None

    self.prev_steer_torque_sensor = None
    self.steer_torque_counter = 0

  def update(self, can_parsers) -> structs.CarState:
    cp_cbp = can_parsers[Bus.main]
    cp_rdr = can_parsers[Bus.radar]
    cp_ibst = can_parsers[Bus.adas]
    ret = structs.CarState()

    ret.vEgoRaw = cp_cbp.vl["CBP_status"]["VEGO"] / 3.6 # km/h
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    self.rpm.update(cp_cbp.vl["CBP_status"]["RPM"])
    ret.engineRpm = self.rpm.x

    ret.gearShifter = GearShifter.drive

    self.steer_angle = cp_rdr.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
    self.steer_fraction = cp_rdr.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    self.steer_rate = cp_rdr.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    ret.steeringAngleDeg = -1 * (self.steer_angle + self.steer_fraction)
    ret.steeringRateDeg = -1 * self.steer_rate
    ret.steeringTorque = cp_cbp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp_cbp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * 88.0
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    torque_sensor_angle_deg = cp_cbp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    if cp_cbp.vl["STEER_TORQUE_SENSOR"] != self.prev_steer_torque_sensor:
      self.steer_torque_counter = (self.steer_torque_counter + 1) % 16
    self.prev_steer_torque_sensor = copy.copy(cp_cbp.vl["STEER_TORQUE_SENSOR"])

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp_cbp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
     self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      ret.steeringAngleDeg = torque_sensor_angle_deg
      ret.steeringRateDeg = self.angle_rate_calculator.update(ret.steeringAngleDeg, self.steer_torque_counter)

    ret.steerFaultTemporary = cp_cbp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp_cbp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(150,
      cp_cbp.vl["CBP_status"]["BLINKER_LEFT"],
      cp_cbp.vl["CBP_status"]["BLINKER_RIGHT"]
    )

    cp_cbp.vl["CBP_status"]["BLINKER_LEFT"] == 1
    ret.rightBlinker = cp_cbp.vl["CBP_status"]["BLINKER_RIGHT"] == 1

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cp_cbp.vl["CBP_status"]["ENGAGED"] == 1
    ret.cruiseState.speed = cp_cbp.vl["CBP_status"]["SET_SPEED"] / 3.6 #1 km/h

    ret.brake = cp_ibst.vl["IBST_status"]["IBST_brakeInputStroke"]
    ret.brakePressed = cp_ibst.vl["IBST_private2"]["IBST_brakePedalApplied"] != 0

    # TODO: do we need this?
    ret.gas = 0
    ret.gasPressed = False
    ret.seatbeltUnlatched = False
    ret.doorOpen = False

    return ret

  @staticmethod
  def get_can_parsers(CP):
    main_msgs = [
      ("CBP_status", 10),
      ("EPS_STATUS", 25),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    steer_sensor_msgs = [
      ("STEER_ANGLE_SENSOR", 80),
    ]

    ibst_msgs = [
      ("IBST_status", 25),
      ("IBST_private2", 50),
    ]

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], main_msgs, 0),
      Bus.radar: CANParser(DBC[CP.carFingerprint][Bus.main], steer_sensor_msgs, 1),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], ibst_msgs, 2)
    }
