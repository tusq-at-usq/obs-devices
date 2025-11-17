from __future__ import annotations
import os
import subprocess
import warnings
import numpy as np
from PIL import Image
import pandas as pd
from scipy.optimize import minimize
from astropy.io import fits
from astropy.table import Table
from astropy.coordinates import EarthLocation, SkyCoord
from astropy.time import Time
from astropy import units as u
from astropy.coordinates import AltAz
from numpy.typing import ArrayLike, NDArray
import cv2
# from scipy.spatial.transform import Rotation as R

# from scoti.utils import gimbal_to_cam_rot
# from airtools.functs import pinhole_mat_from_fov
# from airtools.calibrate import (
#     calc_rotation_from_vectors,
#     check_rotation_cal,
# )

import astrix as at
from astrix.spatial import Rotation


class PlateSolveSolution:
    pass


class PlateData:
    azel_ned: np.ndarray
    uv: np.ndarray
    azel_gimb: np.ndarray
    t: np.ndarray
    n: int

    def __init__(
        self,
        azel_ned: ArrayLike = np.empty((0, 2)),
        uv: ArrayLike = np.empty((0, 2)),
        azel_gimb: ArrayLike = np.empty((0, 2)),
        t_unix: ArrayLike = np.empty((0,)),
    ):
        self.azel_ned = np.array(azel_ned)
        self.uv = np.array(uv)
        self.t = np.array(t_unix)
        self.n = self.azel_ned.shape[0]

        azel_gimb = np.array(azel_gimb)
        t = np.array(t_unix)
        if azel_gimb.shape[0] == 1 and self.azel_ned.shape[0] > 1:
            self.azel_gimb = np.full((self.n, 2), self.azel_gimb[0])
            if t.shape[0] == 1:
                self.t = np.full((self.n,), t[0])
        if not (
            self.azel_gimb.shape[0]
            == self.azel_ned.shape[0]
            == self.uv.shape[0]
            == self.t.shape[0]
        ):
            raise ValueError("Plate data lengths do not match")

    def __add__(self, new: PlateData):
        self.azel_ned = np.vstack([self.azel_ned, new.azel_ned])
        self.uv = np.vstack([self.uv, new.uv])
        self.azel_gimb = np.vstack([self.azel_gimb, new.azel_gimb])
        self.t = np.hstack([self.t, new.t])
        self.n = self.n + new.n
        return self

    def __getitem__(self, i):
        return self.__class__(
            self.azel_ned[i],
            self.uv[i],
            self.azel_gimb[i],
            self.t[i],
        )

    def save(self, filepath):
        df = pd.DataFrame(
            {
                "az_ned": self.azel_ned[:, 0],
                "el_ned": self.azel_ned[:, 1],
                "u": self.uv[:, 0],
                "v": self.uv[:, 1],
                "az_gimbal": self.azel_gimb[:, 0],
                "el_gimbal": self.azel_gimb[:, 1],
                "time_unix": self.t,
            }
        )
        df.to_csv(filepath)


class PlateSolve:
    pt: at.Point
    dir: str
    _earth_loc: EarthLocation
    _settings: dict
    _idx: int

    _plate_data: PlateData

    def __init__(
        self,
        pt: at.Point,
        working_dir="~/plate_solve_data",
        scale_high: int = 7,
        scale_low: int = 2,
        downsample: int = 2,
        opts: list[str] = [],
        clear_dir: bool = False,
    ):
        self.dir = os.path.join(os.getcwd(), working_dir)
        if clear_dir:
            os.system("rm -rf " + self.dir)
        if not os.path.exists(self.dir):
            os.makedirs(self.dir)

        self.pt = pt
        self._earth_loc = EarthLocation(
            lat=self.pt.geodet[0, 0],
            lon=self.pt.geodet[0, 1],
            height=self.pt.geodet[0, 2] * u.m,  # pyright: ignore
        )

        self._settings = {
            "scale_high": scale_high,
            "scale_low": scale_low,
            "downsample": downsample,
            "opts": opts,
        }
        self._plate_data = PlateData()
        self._idx = 0

    def __call__(
        self,
        image: NDArray,
        azel_gimb: ArrayLike = [0, 0],
        t_unix: float | None = None,
        confirm: bool = True,
    ):
        if t_unix is None:
            warnings.warn(
                "Using current time instead of image time. \
            This is very approximate"
            )
            observ_time = Time.now()
        else:
            observ_time = Time(t_unix, format="unix")

        # Save image in working directory and keep filepath
        p = Image.fromarray(image)
        image_filepath = str(self.dir + "/ps_" + str(self.idx) + ".png")
        p.save(image_filepath)

        init_strings = [
            "solve-field",
            "--scale-low " + str(self._settings["scale_low"]),
            "--scale-high " + str(self._settings["scale_high"]),
            "--scale-units degwidth",
            "--downsample " + str(self._settings["downsample"]),
            "--crpix-center",
            "--overwrite",
            "--resort",
            "--cpulimit 30",
        ]
        try:
            file_strings = [image_filepath, "-D", self.dir]
            command_string = " ".join(
                (init_strings + self._settings["opts"] + file_strings)
            )
            subprocess.run(command_string, shell=True)
            self.idx += 1

            corr_filepath = image_filepath.split(".")[0] + ".corr"
            corr = Table(fits.open(corr_filepath)[1].data)  # pyright: ignore

            # TODO: Can I also extract centerpoint ra, dec here?
            uv = np.array([corr["field_x"], corr["field_y"]], dtype=float).T
            ra_dec = np.array([corr["field_ra"], corr["field_dec"]], dtype=float).T
            aa = AltAz(
                location=self._earth_loc,
                obstime=observ_time,
                pressure=101325 * u.Pa,  # pyright: ignore
                temperature=20 * u.deg_C,
                relative_humidity=0.5,
            )
            sky_coords = SkyCoord(ra_dec[:, 0], ra_dec[:, 1], unit="deg")
            az_el = sky_coords.transform_to(aa)

            plate_data = PlateData(
                [az_el.az.degree, az_el.alt.degree],  # pyright: ignore
                uv,
                azel_gimb,
                np.array(observ_time.unix),
            )

            # TODO: Return solved field to display
            index_img_filepath = image_filepath.split(".")[0] + "-indx.png"
            index_image = np.array(Image.open(index_img_filepath))

            if confirm:
                print(f"Plate solve successful: {plate_data.n} stars found")
                cv2.imshow("Solved Image", image)
                print("Press (y) to confirm, (n) to reject")
                key = cv2.waitKey(0)
                cv2.destroyAllWindows()
                if key == ord("y"):
                    self._plate_data += plate_data
                    return {
                        "status": 1,
                        "plate_data": plate_data,
                        "index_img": index_image,
                    }
                else:
                    print("--- PLATE SOLVE REJECTED: Try with new image ---")
                    return {
                        "status": 0,
                        "plate_data": PlateData(),
                        "index_img": None,
                    }

            return {
                "status": 1,
                "plate_data": plate_data,
                "index_img": index_image,
            }

        except Exception as e:
            print(e)
            print("--- PLATE SOLVE FAILED: Try with new image ---")
            return {
                "status": 0,
                "plate_data": PlateData(),
                "index_img": None,
            }


def solve_angle(plate_data: PlateData, pt: at.Point, res: tuple, fov_0: float = 4):
    # def pinhole_mat_from_fov(fov: float, res: tuple | list) -> NDArray:
    #     """Generate pinhole camera matrix from field of view and resolution."""
    #     f_x = res[0] / (2 * np.tan(np.radians(fov) / 2))
    #     f_y = res[1] / (2 * np.tan(np.radians(fov) / 2))
    #     c_x = res[0] / 2
    #     c_y = res[1] / 2
    #     return np.array([[f_x, 0, c_x], [0, f_y, c_y], [0., 0., 1.]])

    ned_frame = at.spatial.frame.ned_frame(pt)
    time = at.Time(plate_data.t)
    rots = Rotation.from_euler(
        "ZYX",
        np.array(
            [
                plate_data.azel_gimb[:, 0],
                plate_data.azel_gimb[:, 1],
                np.zeros(plate_data.n),
            ]
        ).T,
        degrees=True,
    )
    rot_seq = at.RotationSequence(rots, time)
    frame = at.Frame(rot_seq, ref_frame=ned_frame)
    pixels = at.Pixel(plate_data.uv, at.Time(plate_data.t))

    def compute_origin_rotation(fov: float) -> tuple[float, NDArray, NDArray]:
        cam = at.FixedZoomCamera.from_hoz_fov(res, fov, (1, 1))
        ray = at.Ray.from_camera(pixels, cam, frame)
        vec_gimbal_ned = ray.to_frame(ned_frame).unit_rel

        ned_rots = Rotation.from_euler(
            "ZYX",
            np.array(
                [
                    plate_data.azel_ned[:, 0],
                    plate_data.azel_ned[:, 1],
                    np.zeros(plate_data.n),
                ]
            ).T,
            degrees=True,
        )
        vec_ned = (ned_rots.as_matrix() @ np.array([1, 0, 0])).T
        vec_ned = vec_ned / np.linalg.norm(vec_ned, axis=0)

        # Solve for rotation solution (Wahba's equation)
        rot_mat, angle_delta = at.utils.solve_wahba(vec_gimbal_ned, vec_ned)
        rot_sol = Rotation.from_matrix(rot_mat)
        error_rms = np.linalg.norm(angle_delta)

        return float(error_rms), rot_sol.as_euler("ZYX", degrees=True), angle_delta

    def opt_func(fov_arr: NDArray) -> float:
        error_rms, _, _ = compute_origin_rotation(fov_arr[0])
        return error_rms

    def eval_func(fov: float) -> tuple[NDArray, NDArray]:
        _, rot_sol, angle_delta = compute_origin_rotation(fov)
        return rot_sol, angle_delta

    sol = minimize(opt_func, np.array([fov_0]))
    fov_best = float(sol.x[0])
    euler_angles, angle_delta = eval_func(fov_best)
    return euler_angles, fov_best, angle_delta
