"""Planar turning-centre hypothesis applied to an existing EKF trajectory.

The 0.2159 m candidate was selected after inspecting run2: its effective
zero-lateral-velocity point was close to the SMB front axle. This is a frozen
kinematic hypothesis for evaluation on NEW routes, not a corrected URDF
extrinsic or a parameter observable from wheel speed and gyro alone.

No measurement is fused here. The original EKF output is reinterpreted as a
virtual point C, with body origin B situated ``offset_m`` behind C. Initial
translation is aligned so the first output keeps the original position.
Only planar motion is modelled; this adapter does not estimate 3-D slip.
"""
from __future__ import annotations

from copy import deepcopy
import math
from typing import Any

import numpy as np


FRONT_AXLE_HYPOTHESIS_M = 0.2159


def _vector(row: dict[str, Any], name: str, size: int) -> np.ndarray:
    value = np.asarray(row[name], dtype=float)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return value


class TurningCenterAdapter:
    """Transform chronological ``odom_row`` dictionaries without mutating them.

    Pose covariance order is [x,y,z,roll,pitch,yaw]; twist covariance order is
    [vx,vy,vz,wx,wy,wz]. Jacobians retain all existing cross-covariances. The
    result is conditional on the fixed offset and first-heading anchor: it
    does not include uncertainty in the slip model or the stored anchor, nor
    the unavailable cross-time covariance with that anchor. It is therefore
    not a calibrated confidence bound for the candidate motion model.
    """

    def __init__(self, offset_m: float = FRONT_AXLE_HYPOTHESIS_M):
        self.offset_m = float(offset_m)
        if not math.isfinite(self.offset_m):
            raise ValueError("offset_m must be finite")
        self.initial_yaw_rad: float | None = None

    def convert(self, odom_row: dict[str, Any]) -> dict[str, Any]:
        """Keep timestamps, frame names, orientation and unmodified fields."""
        position = _vector(odom_row, "position", 3)
        quaternion = _vector(odom_row, "quaternion", 4)
        linear = _vector(odom_row, "linear", 3)
        angular = _vector(odom_row, "angular", 3)
        pose_cov = _vector(odom_row, "pose_covariance", 36).reshape(6, 6)
        twist_cov = _vector(odom_row, "twist_covariance", 36).reshape(6, 6)
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.e-12:
            raise ValueError("quaternion must have nonzero norm")
        x, y, z, w = quaternion / norm
        yaw = math.atan2(2. * (w*z + x*y), 1. - 2. * (y*y + z*z))
        if self.initial_yaw_rad is None:
            self.initial_yaw_rad = yaw

        result = deepcopy(odom_row)
        ell = self.offset_m
        if ell == 0.:
            return result

        # p_B = p_C + R(yaw_0)[ell,0] - R(yaw)[ell,0].
        # Body velocity follows v_B = v_C + omega x [-ell,0,0].
        corrected_position = position.copy()
        corrected_position[0] += ell * (math.cos(self.initial_yaw_rad) - math.cos(yaw))
        corrected_position[1] += ell * (math.sin(self.initial_yaw_rad) - math.sin(yaw))
        corrected_linear = linear.copy()
        corrected_linear[1] -= ell * angular[2]
        result["position"] = corrected_position.tolist()
        result["linear"] = corrected_linear.tolist()

        # Propagate the same posterior; do not feed gyro into a second update.
        pose_jacobian = np.eye(6)
        pose_jacobian[0, 5] = ell * math.sin(yaw)
        pose_jacobian[1, 5] = -ell * math.cos(yaw)
        twist_jacobian = np.eye(6)
        twist_jacobian[1, 5] = -ell
        result["pose_covariance"] = (
            pose_jacobian @ pose_cov @ pose_jacobian.T).reshape(-1).tolist()
        result["twist_covariance"] = (
            twist_jacobian @ twist_cov @ twist_jacobian.T).reshape(-1).tolist()
        return result


def check_invariants() -> dict[str, bool]:
    """Small numerical checks of identity, rigid planar motion and covariance."""
    ell = FRONT_AXLE_HYPOTHESIS_M
    # Correlated, positive-definite input exercises off-diagonal propagation.
    factor = np.eye(6)
    factor[0, 5], factor[1, 5] = .2, -.1
    covariance = (factor @ factor.T).reshape(-1).tolist()
    row = {"stamp_ns": 0, "frame": "shadow_odom", "child": "base_link",
           "position": [1., 2., 0.], "quaternion": [0., 0., 0., 1.],
           "linear": [0., 0., 0.], "angular": [0., 0., .3],
           "pose_covariance": covariance, "twist_covariance": list(covariance)}
    original = deepcopy(row)
    identity = TurningCenterAdapter(0.).convert(row)
    assert identity == row and identity is not row
    identity["position"][0] = 100.
    assert row == original

    adapter = TurningCenterAdapter(ell)
    first = adapter.convert(row)
    np.testing.assert_allclose(first["position"], row["position"], atol=1.e-14)
    rotated = deepcopy(row)
    rotated["stamp_ns"] = 1_000_000_000
    rotated["quaternion"] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
    output = adapter.convert(rotated)
    np.testing.assert_allclose(output["position"], [1. + ell, 2. - ell, 0.], atol=1.e-14)
    np.testing.assert_allclose(output["linear"], [0., -.3 * ell, 0.], atol=1.e-14)
    assert output["stamp_ns"] == rotated["stamp_ns"]
    assert output["frame"] == row["frame"] and output["child"] == row["child"]
    assert row == original
    for name in ("pose_covariance", "twist_covariance"):
        matrix = np.asarray(output[name]).reshape(6, 6)
        np.testing.assert_allclose(matrix, matrix.T, atol=1.e-14)
        assert np.linalg.eigvalsh(matrix).min() >= -1.e-12
    return {"zero_offset_identity": True, "first_position_preserved": True,
            "pure_rotation_position_and_velocity": True,
            "covariance_symmetric_positive_semidefinite": True,
            "input_unmodified": True}


if __name__ == "__main__":
    import json
    print(json.dumps(check_invariants(), indent=2))
