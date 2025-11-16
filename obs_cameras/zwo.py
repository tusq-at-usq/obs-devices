import threading
import zwoasi as asi
from typing import Any
import time
import warnings
from obs_cameras.base import CameraInterface, Frame


class ASI585(CameraInterface):
    NAME = "ASI585"
    MODEL_NO = "585"
    FRAME_RES = 3840, 2160
    USB_TRANSFER_TIME: float = 0.001  # Estimated USB transfer time in seconds (1ms)
    DTYPE = "uint8"
    GAIN_DEFAULT = 5
    EXP_DEFAULT = 20e3

    _frame_delivered = threading.Event
    _asicam: asi.Camera
    _controls: dict[str, dict[str, Any]]

    _limits = {}

    def __init__(self):
        super().__init__()
        self.cam_id = None
        self._frame_delivered = threading.Event()

    @property
    def name(self) -> str:
        return self.NAME

    def reconnect(self, idx=0):
        cam_dict, num_cameras = self.list_devices()
        if num_cameras == 0:
            print("No cameras found")
        elif num_cameras > 1:
            print("Setting 'ASI' to num " + str(idx))
        self._asicam = asi.Camera(idx)

    def list_devices(self) -> tuple[dict, int]:
        num_cameras = asi.get_num_cameras()
        cameras_found = asi.list_cameras()
        return cameras_found, num_cameras

    def __enter__(self) -> CameraInterface:
        controls = self._asicam.get_controls()
        self._limits = {
            "exposure": (
                controls["Exposure"]["MinValue"],
                controls["Exposure"]["MaxValue"],
            ),
            "gain": (controls["Gain"]["MinValue"], controls["Gain"]["MaxValue"]),
            "gain_default": controls["Gain"]["DefaultValue"],
            "bandwidth": (controls["BandWidth"]["MinValue"], controls["BandWidth"]["MaxValue"]),
        }

        self.set_exposure(self.EXP_DEFAULT)
        self.set_gain(self.GAIN_DEFAULT)
        self.set_bandwidth("auto")
        self._asicam.stop_exposure()
        self._asicam.set_control_value(14, 1) # Set high speed mode
        self._asicam.start_video_capture()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self._asicam.stop_video_capture()
        self._asicam.close_camera()
        asi.close()

    def _get_frame(self) -> Frame:
        """
        Hardware-specific implementation of frame capture.
        Must be implemented by derived classes.

        Returns:
            Frame: A Frame object containing the image data and metadata
            timestamp: Timestamp when the frame was captured
        """
        try:
            gain = self.gain
            exposure = self.exposure
            frame = self._asicam.capture_video_frame(timeout=1000)
            capture_time = time.time()
            adjusted_time = (
                capture_time - (self.exposure / 2) / 1e6 - self.USB_TRANSFER_TIME
            )
            return Frame(frame, gain, exposure, adjusted_time, self.name)
        except asi.ZWO_ASI_ERROR_TIMEOUT:
            raise RuntimeError(f"Cam: {self.name} Frame capture timed out.")

    def _get_gain(self) -> float:
        """
        Hardware-specific implementation of gain retrieval.
        Must be implemented by derived classes.

        Returns:
            float: Current gain value
        """
        return self._asicam.get_control_value(asi.ASI_GAIN)

    def _get_exposure(self) -> float:
        """
        Hardware-specific implementation of exposure retrieval.
        Must be implemented by derived classes.

        Returns:
            float: Current exposure time in microseconds
        """
        return self._asicam.get_control_value(asi.ASI_EXPOSURE)

    def _set_exposure(self, exp: float) -> None:
        """
        Args:
            exp: Exposure time in microseconds
        """
        if 0 < exp - self.exposure < self._limits["exposure_incr"]:
            exp = self.exposure + self._limits["exposure_incr"]
        elif 0 > exp - self.exposure > -self._limits["exposure_incr"]:
            exp = self.exposure - self._limits["exposure_incr"]

        if not self._limits["exposure"][0] <= exp <= self._limits["exposure"][1]:
            print("Clipping exposure to valid range.")
        exp = max(self._limits["exposure"][0], min(self._limits["exposure"][1], exp))
        self._asicam.set_control_value(asi.ASI_EXPOSURE, int(exp))

    def _set_gain(self, gain: float | str) -> None:
        """
        Args:
            gain: Gain value
        """
        if isinstance(gain, float):
            if 0 < gain - self.gain < self._limits["gain_incr"]:
                gain = self.gain + self._limits["gain_incr"]
            elif 0 > gain - self.gain > -self._limits["gain_incr"]:
                gain = self.gain - self._limits["gain_incr"]

            if not self._limits["gain"][0] <= gain <= self._limits["gain"][1]:
                print("Clipping gain to valid range.")
            gain = max(self._limits["gain"][0], min(self._limits["gain"][1], gain))
            self._asicam.set_control_value(asi.ASI_GAIN, gain)

        elif gain == "auto":
            self._asicam.set_control_value(
                asi.ASI_GAIN,
                self._limits["gain_default"],
                auto=True,
            )
        else:
            warnings.warn("Gain value not recognised; no changes made.")

    def _set_bandwidth(self, bw: float | str) -> None:
        """
        Args:
            bw: Bandwidth limit in Mbps
        """
        if isinstance(bw, float):
            if not self._limits["bandwidth"][0] <= bw <= self._limits["bandwidth"][1]:
                print("Clipping bandwidth to valid range.")
            bw = max(self._limits["bandwidth"][0], min(self._limits["bandwidth"][1], bw))
            self._asicam.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, int(bw))

        if bw == "auto":
            self._asicam.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 80, auto=True)

        if bw == "min":
            self._asicam.set_control_value(
                asi.ASI_BANDWIDTHOVERLOAD,
                int(self._limits["bandwidth"][0])
            )
        elif bw == "min":
            self._asicam.set_control_value(
                asi.ASI_BANDWIDTHOVERLOAD,
                int(self._limits["bandwidth"][1]),
            )
        else:
            warnings.warn("Bandwidth value not recognised; no changes made.")
