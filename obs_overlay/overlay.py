"""
1. Read trajory file in standardised format
2. Downsample trajectory
3. Create rays from current location to downsampled points
4. Project to 2D image plane

"""
import os
import yaml
import time
import numpy as np
import pandas as pd
import astrix as at
from astrix.spatial import Rotation
import jax
from jax import Array, numpy as jnp
from jax.typing import ArrayLike
from functools import partial
from abc import ABC, abstractmethod

def read_varda_traj(file_path, test_time_adjustment=False):
    df = pd.read_csv(os.path.expanduser(file_path))
    t = at.Time(df["Time_From_Separation_s"])
    ecef = np.array([df["ECEF_x_m"], df["ECEF_y_m"], df["ECEF_z_m"]]).T
    pt = at.Point(ecef, t)
    path_nom = at.Path(pt)
    if test_time_adjustment:
        unix_30s = time.time() - path_nom.time_at_alt(110e3).unix[0] + 30 
        t = t.offset(unix_30s)
        pt = at.Point(ecef, t)
        path_nom = at.Path(pt)
    path_nom = path_nom.truncate(
        start_time=path_nom.time_at_alt(100e3), end_time=path_nom.time_at_alt(20e3)
    )
    path_nom = at.utils.defeature_path(path_nom, tol = 50)
    return path_nom

class Overlay(ABC):

    @abstractmethod
    @partial(jax.jit, static_argnames=("self",))
    def project_from_ecef_angles(self, euler: ArrayLike, t_unix: float) -> tuple[ArrayLike, ArrayLike]:
        pass

    @abstractmethod
    @partial(jax.jit, static_argnames=("self",))
    def project_from_ned_angles(self, euler: ArrayLike, t_unix: float) -> tuple[ArrayLike, ArrayLike]:
        pass

    @abstractmethod
    @partial(jax.jit, static_argnames=("self",))
    def check_time_bounds(self, t_unix: float) -> tuple[bool, bool]:
        pass



class PathOverlay(Overlay):
    _point: at.Point
    _target_pt: at.Point
    _target_paths: at.Path
    _ray_ecef: at.Ray
    _frame_ned: at.Frame
    _cam: at.FixedZoomCamera

    def __init__(self, point: at.Point, target: at.Path, cam: at.FixedZoomCamera) -> None:
        self._point = point
        self._target_path = target
        self._target_pt = target.points
        self._cam = cam.convert_to(jnp)
        self._ray_ecef = at.Ray.from_points(self._target_pt, self._point, backend=jnp)
        self._frame_ned = at.spatial.frame.ned_frame(self._point).convert_to(jnp)

    @partial(jax.jit, static_argnames=("self",))
    def project_from_ecef_angles(self, euler: ArrayLike, t_unix: float) -> tuple[ArrayLike, ArrayLike]:
        """Project target points to image plane from ECEF angles.

        Args:
            euler (ArrayLike): Euler angles (yaw, pitch, roll) in radians.
        Returns:
            uv_path: 2D image coordinates of projected points.
            uv_point: 2D image coordinates of interpolated point at t_unix.
        """
        rot = Rotation.from_euler("ZYX", jnp.array(euler).reshape(1,-1) , degrees=1)
        frame = at.Frame(rot, loc=self._point, backend=jnp)
        # frame = at.Frame(rot, ref_frame=self._frame_ned, backend=jnp)
        ray = self._ray_ecef.to_frame(frame)
        uv_path = ray.project_to_cam(self._cam)
        uv_point = ray.interp(at.Time(t_unix, backend=jnp), check_bounds=False).project_to_cam(self._cam)
        return uv_path.uv, uv_point.uv


    @partial(jax.jit, static_argnames=("self",))
    def project_from_ned_angles(self, euler: ArrayLike, t_unix: float) -> tuple[ArrayLike, ArrayLike]:
        """Project target points to image plane from NED angles.

        Args:
            euler (ArrayLike): Euler angles (yaw, pitch, roll) in radians.
        Returns:
            uv_path: 2D image coordinates of projected points.
            uv_point: 2D image coordinates of interpolated point at t_unix.
        """
        rot = Rotation.from_euler("ZYX", jnp.array(euler).reshape(1,-1), degrees=1)
        frame = at.Frame(rot, ref_frame=self._frame_ned, backend=jnp)
        ray = self._ray_ecef.to_frame(frame)
        uv_path = ray.project_to_cam(self._cam)
        uv_point = ray.interp(at.Time(t_unix, backend=jnp), check_bounds=False).project_to_cam(self._cam)
        return uv_path.uv, uv_point.uv

    @partial(jax.jit, static_argnames=("self",))
    def check_time_bounds(self, t_unix: float) -> tuple[bool, bool]:
        """Check if the given unix time is within the bounds of the target path.

        Args:
            t_unix (float): Unix time to check.
        Returns:
            start_in_bounds (bool): True if t_unix is after the start time of the target path.
            end_in_bounds (bool): True if t_unix is before the end time of the target path.
        """
        start_in_bounds = t_unix >= self._target_path.start_time.unix[0]
        end_in_bounds = t_unix <= self._target_path.end_time.unix[0]
        return start_in_bounds, end_in_bounds
            








