#!/usr/bin/env python3
"""Periodic BMS status monitor for /BMS_status.
Prints a compact ASCII dashboard every N seconds.
"""
import math
import threading
import time

import rospy
from scout_msgs.msg import ScoutBmsStatus


class BmsStatusMonitor:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/BMS_status")
        self.period = float(rospy.get_param("~period", 60.0))  # seconds
        self._last_msg = None
        self._last_stamp = None
        self._lock = threading.Lock()

        rospy.loginfo("BMS monitor subscribed to %s, period=%.1fs", self.topic, self.period)
        self._sub = rospy.Subscriber(self.topic, ScoutBmsStatus, self._cb, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(self.period), self._on_timer)

    def _cb(self, msg: ScoutBmsStatus):
        with self._lock:
            self._last_msg = msg
            self._last_stamp = rospy.Time.now()

    def _on_timer(self, _event):
        with self._lock:
            msg = self._last_msg
            stamp = self._last_stamp

        if msg is None:
            rospy.logwarn("No BMS_status received yet from %s", self.topic)
            return

        age = "{:.1f}s".format((rospy.Time.now() - stamp).to_sec()) if stamp else "n/a"
        dash = self._format_dashboard(msg, age)
        print(dash)

    @staticmethod
    def _format_dashboard(msg: ScoutBmsStatus, age_str: str) -> str:
        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        soc = msg.SOC
        soh = msg.SOH
        volt = msg.battery_voltage
        curr = msg.battery_current
        temp = msg.battery_temperature
        alarm1 = msg.Alarm_Status_1
        alarm2 = msg.Alarm_Status_2
        warn1 = msg.Warning_Status_1
        warn2 = msg.Warning_Status_2

        bar_len = 20
        filled = int(round(clamp(soc, 0.0, 100.0) / 100.0 * bar_len))
        bar = "#" * filled + "-" * (bar_len - filled)

        lines = [
            "+---------------------- BMS STATUS -----------------------+",
            "| Topic: {topic:<46}|".format(topic="/BMS_status"),
            "| Age: {age:<48}|".format(age=age_str),
            "+---------------------------------------------------------+",
            "| SOC (%)   : {soc:6.2f}    SOH (%)   : {soh:6.2f}              |".format(soc=soc, soh=soh),
            "| Voltage(V): {volt:6.2f}    Current(A): {curr:6.2f}              |".format(volt=volt, curr=curr),
            "| Temp (C)  : {temp:6.2f}                                   |".format(temp=temp),
            "+---------------------------------------------------------+",
            "| Alarms: 0x{a1:02X} 0x{a2:02X}    Warnings: 0x{w1:02X} 0x{w2:02X}             |".format(
                a1=alarm1, a2=alarm2, w1=warn1, w2=warn2
            ),
            "+---------------------------------------------------------+",
            "| SOC bar: [{bar}] {soc:5.1f}%                          |".format(bar=bar, soc=soc),
            "+---------------------------------------------------------+",
        ]
        return "\n" + "\n".join(lines) + "\n"


if __name__ == "__main__":
    rospy.init_node("bms_status_monitor")
    monitor = BmsStatusMonitor()
    rospy.spin()
