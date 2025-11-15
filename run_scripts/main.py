from obs_cameras.alvium import Alvium811
from obs_cameras.base import CameraStream
from obs_display.display import Display
from obs_overlay.overlay import PathOverlay, read_varda_traj

from obs_certus.monitor import CertusMonitor
from obs_encoders.monitor import EncoderMonitor

import astrix as at


def main():
    path_nom = read_varda_traj(
        "~/varda-w4/planning/data/W4_Nominal_ECEF.csv",
        test_time_adjustment=True,
    )
    pt_GS2 = at.Point.from_geodet([-32.150517, 133.689040, 10])
    alv811_25_mod = at.FixedZoomCamera(
        (2848, 2848), (2.74 * 1e-3 * 2848, 2.74 * 1e-3 * 2848), 21
    )

    alv811 = Alvium811()
    imu = CertusMonitor(az_rot_deg=180, pc_time=1)
    encoders = EncoderMonitor()
    stream = CameraStream("Test_alvium", alv811, "~/test_image_data")
    overlay = PathOverlay(pt_GS2, path_nom, alv811_25_mod)
    display = Display("test_cam", stream, imu, encoders, overlay)

    display.run()


if __name__ == "__main__":
    main()
