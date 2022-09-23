#!/usr/bin/env python3
# a simple steering only driver monitor module for C2

from cereal import car
EventName = car.CarEvent.EventName
from common.realtime import DT_DMON # 0.05 = 20hz

# ref (page15-16): https://eur-lex.europa.eu/legal-content/EN/TXT/PDF/?uri=CELEX:42018X1947&rid=2
_AWARENESS_TIME = 30.  # 30 secs limit without user touching steering wheels make the car enter a terminal status
_AWARENESS_PRE_TIME_TILL_TERMINAL = 15.  # a first alert is issued 15s before expiration
_AWARENESS_PROMPT_TIME_TILL_TERMINAL = 6.  # a second alert is issued 6s before start decelerating the car

class DriverStatus():
  def __init__(self):
    self.terminal_alert_cnt = 0
    self.terminal_time = 0

    self.awareness = 1.

    self.ts_last_check = 0.

    self.threshold_pre = _AWARENESS_PRE_TIME_TILL_TERMINAL / _AWARENESS_TIME
    self.threshold_prompt = _AWARENESS_PROMPT_TIME_TILL_TERMINAL / _AWARENESS_TIME
    self.step_change = DT_DMON / _AWARENESS_TIME

  def update_events(self, events, driver_engaged, ctrl_active, standstill):
    #modal
    self.awareness = 1.
    return


if __name__ == "__main__":
  pass