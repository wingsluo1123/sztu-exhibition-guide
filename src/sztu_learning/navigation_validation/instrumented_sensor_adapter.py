"""Unchanged validated conversion and gates, with per-frame timing evidence."""
import os
from pathlib import Path
import sys

sys.path.insert(0, os.environ['LEARNING_LOCALIZATION_ROOT'])
import rclpy
from online_sensor_adapter import LiveSensorAdapter
from timing_trace import TimingTrace


class TracedAdapter(LiveSensorAdapter):
    def __init__(self, path):
        super().__init__()
        self.trace = TimingTrace(path)

    def receive(self, message):
        super().receive(message)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        self.trace.record('lidar', stamp, input_age_s=self.last_age,
                          conversion_s=self.last_duration, reason=self.last_reason,
                          accepted=self.accepted, rejected=self.rejected)


def main():
    rclpy.init()
    node = TracedAdapter(Path(sys.argv[1]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trace.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
