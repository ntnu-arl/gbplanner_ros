"""One-shot handoff to an external landing service; no landing dynamics.

The planner owns the ROS-clock deadline. This helper only latches its intent
and bounds cancellation/service waits with the controller's monotonic clock.
An Empty response proves transport completion, never physical touchdown.
"""
import math


class LandingPolicy:
    def __init__(self, robot_type, timeout):
        if robot_type not in {'kAerial', 'kGround'} or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Invalid landing policy configuration')
        self.robot_type, self.timeout = robot_type, float(timeout)
        self.configured = self.enabled = self.clock_valid = self.latched = False
        self.phase, self.reason = 'DISABLED', 'disabled'
        self.remaining = None
        self.deadline = 0.
        self.cancellation_required = self.cancellation_confirmed = False
        self.requests = 0
        self.response_received = self.late_response_received = False

    def observe(self, mission, now):
        """Return true only when a valid aerial deadline is first latched."""
        data = mission.get('landing')
        if not isinstance(data, dict):
            return False
        keys = ('configured', 'enabled', 'clock_valid', 'due')
        if any(not isinstance(data.get(key), bool) for key in keys):
            return False
        remaining = data.get('remaining_s')
        if remaining is not None and (isinstance(remaining, bool) or
                                      not isinstance(remaining, (int, float)) or
                                      not math.isfinite(remaining) or remaining < 0):
            return False
        if data['enabled'] and not data['configured']:
            return False
        if self.latched:
            return False  # Repeated status, map resets and disable messages cannot rearm.
        self.configured = data['configured']
        self.enabled = data['enabled'] and self.robot_type == 'kAerial'
        self.clock_valid, self.remaining = data['clock_valid'], remaining
        self.phase = 'ARMED' if self.enabled else 'DISABLED'
        self.reason = ('waiting_for_deadline' if self.enabled else
                       'ground_robot' if self.robot_type == 'kGround' and self.configured else 'disabled')
        if not (self.enabled and self.clock_valid and data['due']):
            return False
        self.latched = True
        self.phase, self.reason = 'QUIESCING', 'auto_landing_due'
        self.deadline = now + self.timeout
        return True

    def expect_cancellation(self, required):
        self.cancellation_required = bool(required)
        self.cancellation_confirmed = not required

    def motion_result(self, confirmed):
        if self.phase != 'QUIESCING':
            return
        self.cancellation_confirmed = bool(confirmed)
        if not confirmed:
            self.phase, self.reason = 'UNCONFIRMED', 'landing_cancellation_unconfirmed'

    def advance(self, now, motion_clear, odometry_fresh, service_ready):
        """Return true exactly once, when the caller should send Empty."""
        if self.phase == 'QUIESCING':
            if now > self.deadline:
                if not self.cancellation_confirmed or not motion_clear:
                    self.phase, self.reason = 'UNCONFIRMED', 'landing_cancellation_unconfirmed'
                elif not odometry_fresh:
                    self.phase, self.reason = 'UNCONFIRMED', 'landing_odometry_unavailable'
                else:
                    self.phase, self.reason = 'FAILED', 'landing_service_unavailable'
            elif self.cancellation_confirmed and motion_clear and odometry_fresh and service_ready:
                self.phase, self.reason = 'REQUESTING', 'landing_service_requested'
                self.requests += 1
                self.deadline = now + self.timeout
                return True
        elif self.phase == 'REQUESTING' and now > self.deadline:
            self.phase, self.reason = 'UNCONFIRMED', 'landing_service_timeout'
        return False

    def service_result(self, received, now):
        if self.phase == 'REQUESTING' and now > self.deadline:
            self.phase, self.reason = 'UNCONFIRMED', 'landing_service_timeout'
        if self.phase != 'REQUESTING':
            if received and self.requests:
                self.late_response_received = True
            return
        if received:
            self.response_received = True
            self.phase, self.reason = 'ACKNOWLEDGED', 'landing_service_response_received'
        else:
            self.phase, self.reason = 'UNCONFIRMED', 'landing_service_transport_error'

    def payload(self):
        return {
            'configured': self.configured, 'enabled': self.enabled,
            'clock_valid': self.clock_valid, 'remaining_s': self.remaining,
            'latched': self.latched, 'phase': self.phase, 'reason': self.reason,
            'service': 'land_srv', 'requests': self.requests,
            'cancellation_required': self.cancellation_required,
            'cancellation_confirmed': self.cancellation_confirmed,
            'service_response_received': self.response_received,
            'late_service_response_received': self.late_response_received,
            'physical_confirmation': 'not_observed',
        }
