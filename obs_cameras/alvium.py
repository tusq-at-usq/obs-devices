import time
import vmbpy
import threading
from .base import CameraInterface, Frame


class Alvium811(CameraInterface):
    NAME = "Alvium_811"
    MODEL_NO = "811"
    FRAME_RES = (2848, 2848)
    DTYPE = "uint8"
    GAIN_DEFAULT = 1
    EXP_DEFAULT = 20e3

    _vmb: vmbpy.VmbSystem
    _vmbcam: vmbpy.Camera
    cam_id: str | None
    _frame: vmbpy.Frame
    _frame_delivered = threading.Event

    _limits = {}

    def __init__(self):
        super().__init__()
        self.cam_id = None
        self._frame_delivered = threading.Event()


    @property
    def name(self) -> str:
        return self.NAME

    def reconnect(self):
        cam_dict = self.list_devices()
        vmbcam = None

        if self.cam_id is None:
            try:
                vmbcam = [
                    cam
                    for cam in cam_dict.keys()
                    if self.MODEL_NO in cam_dict[cam]["Model"]
                ][0]
            except IndexError:
                raise RuntimeError(f"No Alvium {self.MODEL_NO} camera found.")
        else:
            try:
                vmbcam = [
                    cam for cam in cam_dict.keys() if self.cam_id in cam_dict[cam]["ID"]
                ][0]
            except IndexError:
                raise RuntimeError(f"Camera with ID {self.cam_id} not found.")
        if vmbcam is not None:
            self._vmbcam = vmbcam
            self.cam_id = cam_dict[vmbcam]["ID"]

    def list_devices(self) -> dict:
        cam_dict = {}
        cams = self._vmb.get_all_cameras()
        for cam in cams:
            try:
                with cam:
                    cam_dict[cam] = {"Model": cam.get_model(), "ID": cam.get_id()}
            except vmbpy.error.VmbCameraError:
                pass
        return cam_dict

    def __enter__(self) -> CameraInterface:
        """Enter the runtime context related to this object."""
        _vmb = vmbpy.VmbSystem.get_instance()
        self._vmb = _vmb.__enter__()
        time.sleep(0.1)

        self.reconnect()
        if self._vmbcam is not None:
            self._vmbcam.__enter__()
        else:
            self.__exit__(None, None, None)
            raise RuntimeError("Failed to connect to camera.")

        self._vmbcam.stop_streaming()
        self._vmbcam.set_pixel_format(vmbpy.PixelFormat.Mono8)
        self._vmbcam.DeviceLinkThroughputLimit.set(400e6)
        self._limits = {
            "exposure": self._vmbcam.ExposureTime.get_range(),
            "exposure_incr": self._vmbcam.ExposureTime.get_increment(),
            "gain": self._vmbcam.Gain.get_range(),
            "gain_incr": self._vmbcam.Gain.get_increment(),
        }
        self.set_exposure(self.EXP_DEFAULT)
        self.set_gain(self.GAIN_DEFAULT)
        self.start_video()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Exit the runtime context related to this object."""
        if self._vmbcam is not None:
            self._vmbcam.stop_streaming()
            self._vmbcam.TriggerMode.set("Off")
            self._vmbcam.AcquisitionMode.set("SingleFrame")
            self._vmbcam.__exit__(exc_type, exc_val, exc_tb)
            print(f"Alvium {self.MODEL_NO} camera gracefully disconnected.")
        self._vmb.__exit__(exc_type, exc_val, exc_tb)

    def start_video(self) -> None:
        """
        Start video capture mode.
        Must be implemented by derived classes.
        """

        # Think this is dodgy? Assigning to self when that isn't a function argument?
        def frame_handler(cam: vmbpy.Camera, stream: vmbpy.Stream, frame: vmbpy.Frame):
            self._frame = frame.as_numpy_ndarray()
            self._frame_delivered.set()
            cam.queue_frame(frame)

        self._vmbcam.stop_streaming()
        self._vmbcam.TriggerSource.set("Software")
        self._vmbcam.TriggerSelector.set("FrameStart")
        self._vmbcam.TriggerMode.set("On")
        self._vmbcam.AcquisitionMode.set("Continuous")
        self._vmbcam.start_streaming(frame_handler)

    def _get_frame(self) -> Frame:
        """
        Hardware-specific implementation of frame capture.
        Must be implemented by derived classes.

        Returns:
            Frame: A Frame object containing the image data and metadata
            timestamp: Timestamp when the frame was captured
        """
        gain = self.gain
        exposure = self.exposure
        self._frame_delivered.clear()
        timestamp = time.time()
        self._vmbcam.TriggerSoftware.run()
        if self._frame_delivered.wait(timeout=2.0):
            frame = Frame(
                pixels=self._frame,
                gain=gain,
                exposure=exposure,
                timestamp=timestamp,
                cam_name=self.name,
            )
            return frame
        else:
            raise RuntimeError("Frame capture timed out.")

    def _get_gain(self) -> float:
        """
        Hardware-specific implementation of gain retrieval.
        Must be implemented by derived classes.

        Returns:
            float: Current gain value
        """
        return self._vmbcam.Gain.get()

    def _get_exposure(self) -> float:
        """
        Hardware-specific implementation of exposure retrieval.
        Must be implemented by derived classes.

        Returns:
            float: Current exposure time in microseconds
        """
        return self._vmbcam.ExposureTime.get()

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
        self._vmbcam.ExposureTime.set(exp)

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
        self._vmbcam.Gain.set(gain)


class Alvium508(Alvium811):
    NAME = "Alvium_508"
    MODEL_NO = "508"
    FRAME_RES = (2464, 2056)
