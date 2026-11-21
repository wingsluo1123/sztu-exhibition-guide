"""Longer stationary acquisition, unchanged tracking/freshness criteria."""
from continuous_localizer import ContinuousLocalizer


class NavigationTracker(ContinuousLocalizer):
    initial_acquisition_s=15.

    def status(self,now):
        # Acquisition has no valid pose and therefore cannot authorize motion.
        # The inherited 5 s deadline expired before the first unambiguous fresh
        # result (5.6 s) in goal_03. Give acquisition time, never stale data.
        if self.state=='initializing' and self.stamp is not None:
            if now-self.stamp>self.config.maximum_input_age_s:
                self.lose('input_timeout_requires_reinitialization')
            elif now-self.started_stamp>self.initial_acquisition_s:
                self.lose('initial_match_timeout_requires_reinitialization')
            return {'state':self.state,'reason':self.reason,'valid':False,'stamp':self.stamp,
                    'last_map_match_stamp':None,'pose_xy_yaw':None,
                    'initial_acquisition_limit_s':self.initial_acquisition_s,
                    'reference_point':'Floor projection of chest IMU origin; heading is projected IMU +Y, not base_link',
                    'recovery':'Restart with verified pose; no automatic motion recovery'}
        return super().status(now)
