import threading
import PyQt6
from PyQt6 import QtGui
import pyqtgraph as pg
import numpy as np
import cv2
import warnings
from dataclasses import dataclass
import datetime
from contextlib import nullcontext

from obs_cameras.base import CameraStream
from obs_encoders.monitor import EncoderMonitor
from obs_certus.monitor import CertusMonitor
from obs_overlay.overlay import Overlay


@dataclass
class DisplaySettings:
    clahe_enabled: bool = False
    clahe_cliplimit: float = 2.0
    clahe: cv2.CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16, 16))
    norm_enabled: bool = False
    colourmap_enabled: bool = False
    hist_enabled: bool = False


class Display:
    _instance_counter = 0
    instance_number: int
    name: str
    _stream: CameraStream
    _kill_event: threading.Event
    _init_labels_event: threading.Event
    _stream_lock: threading.RLock

    app: PyQt6.QtWidgets.QApplication
    win: pg.GraphicsLayoutWidget
    p1: pg.ViewBox
    img: pg.ImageItem
    labs: dict
    x_rules: pg.PlotDataItem
    y_rules: pg.PlotDataItem
    _scale_factor: float
    _display_res: tuple[int, int]
    _width: int

    _disp_set: DisplaySettings

    _imu_monitor: CertusMonitor | None
    _encoder_monitor: EncoderMonitor | None
    _has_imu: bool
    _has_encoder: bool
    _overlay: Overlay | None
    _has_overlay: bool


    def __init__(
        self,
        name,
        stream: CameraStream,
        imu_monitor: CertusMonitor | None = None,
        encoder_monitor: EncoderMonitor | None = None,
        overlay: Overlay | None = None,
        width: int = 1920,
    ):
        Display._instance_counter += 1
        self.instance_number = Display._instance_counter
        self.name = name
        self._stream = stream
        self._imu_monitor = imu_monitor if imu_monitor is not None else nullcontext()
        self._encoder_monitor = encoder_monitor if encoder_monitor is not None else nullcontext()
        self._overlay = overlay
        self._has_imu = imu_monitor is not None
        self._has_encoder = encoder_monitor is not None
        self._has_overlay = overlay is not None
        self._stream_lock = threading.RLock()
        self._kill_event = threading.Event()
        self._disp_set = DisplaySettings()
        self._width = width

        self.app = pg.Qt.mkQApp(name="Video-stream")
        self.win = pg.GraphicsLayoutWidget()
        self.win.setWindowTitle("Display " + str(self.instance_number))
        self.win.keyPressEvent = self.on_key

        self.p1 = self.win.addViewBox(row=0, col=0)
        self.p1.setAspectLocked()

        img = pg.ImageItem()
        hist = pg.HistogramLUTItem(img, orientation="vertical")
        exp_lab = pg.TextItem()
        gain_lab = pg.TextItem()
        fps_lab = pg.TextItem()
        clahe_lab = pg.TextItem()
        saving_lab = pg.TextItem()
        save_queue = pg.TextItem()
        time_lab = pg.TextItem()
        gimb_angles_lab = pg.TextItem()
        imu_angles_lab = pg.TextItem()
        orient_cal_lab = pg.TextItem()
        track_alt_lab = pg.TextItem()

        self.win.addItem(hist, 0, 1)
        self.p1.addItem(img)
        self.p1.addItem(exp_lab)
        self.p1.addItem(gain_lab)
        self.p1.addItem(fps_lab)
        self.p1.addItem(clahe_lab)
        self.p1.addItem(saving_lab)
        self.p1.addItem(save_queue)
        self.p1.addItem(time_lab)
        self.p1.addItem(gimb_angles_lab)
        self.p1.addItem(imu_angles_lab)
        self.p1.addItem(orient_cal_lab)
        self.p1.addItem(track_alt_lab)

        self.img = img
        self.labs = {
            "Exp": exp_lab,
            "Gain": gain_lab,
            "FPS": fps_lab,
            "CLAHE": clahe_lab,
            "Saving": saving_lab,
            "Save queue": save_queue,
            "Time": time_lab,
            "GIM": gimb_angles_lab,
            "IMU": imu_angles_lab,
            "TrackAlt": track_alt_lab,
        }

        self.x_rules = pg.PlotDataItem()
        self.y_rules = pg.PlotDataItem()
        self.p1.addItem(self.x_rules)
        self.p1.addItem(self.y_rules)

        self.offset = 150
        self.traj_bounds_upper = pg.PlotDataItem()
        self.traj_bounds_lower = pg.PlotDataItem()
        self.traj_time_indicator = pg.PlotDataItem()
        self.p1.addItem(self.traj_bounds_upper)
        self.p1.addItem(self.traj_bounds_lower)
        self.p1.addItem(self.traj_time_indicator)

        self.set_bounds()
        self._init_labels()

    def on_key(self, ev: QtGui.QKeyEvent):
        txt = ev.text()
        print(f"Key pressed: {txt}")
        if txt in ["s", "S"]:
            self._stream.save_enabled = not self._stream.save_enabled
        if txt == "G":
            old_gain = self._stream.cam.gain
            new_gain = old_gain * 1.1
            self._stream.cam.set_gain(new_gain)
        elif txt == "g":
            old_gain = self._stream.cam.gain
            new_gain = old_gain * 0.9
            self._stream.cam.set_gain(new_gain)
        elif txt == "E":
            old_exp = self._stream.cam.exposure
            new_exp = old_exp * 1.1
            self._stream.cam.set_exposure(new_exp)
        elif txt == "e":
            old_exp = self._stream.cam.exposure
            new_exp = old_exp * 0.9
            self._stream.cam.set_exposure(new_exp)
        elif txt in ["C", "c"]:
            self.set_clahe(not self._disp_set.clahe_enabled)
        elif txt in ["N", "n"]:
            self.set_norm(not self._disp_set.norm_enabled)
        elif txt in ["M", "m"]:
            self.set_colourmap(not self._disp_set.colourmap_enabled)
        elif txt in ["Q", "q"]:
            self.close()




    def set_bounds(self):
        # self.img_size = self._stream.cam.frame_res
        aspect = self._stream.cam.frame_res[0] / self._stream.cam.frame_res[1]
        height = self._width / aspect
        self._display_res = (int(self._width), int(height))
        self._scale_factor = self._display_res[0] / self._stream.cam.frame_res[0]


        # Set constant bounds for the view
        self.p1.setRange(
            xRange=(0, self._display_res[0]), yRange=(0, self._display_res[1]), padding=0
        )

        # Disable auto-scaling
        self.p1.disableAutoRange()

        # Lock aspect ratio (optional)
        self.p1.setAspectLocked(True)

    def downscale(self, img):
        if self._scale_factor != 1.0:
            new_size = (
                int(img.shape[1] * self._scale_factor),
                int(img.shape[0] * self._scale_factor),
            )
            img_resized = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
            return img_resized
        else:
            return img

    def _init_labels(self):
        img_ratio = self._display_res[0] / self._display_res[1]
        win_dims = (int(self._width * 0.8), int(self._width * 0.8 * (1 / img_ratio) - 100))
        self.win.resize(*win_dims)

        # Private re-scale function just for labels
        def rescale(x, y):
            if x > 0:
                x_ = x * self._display_res[0] / 1920
            else:
                x_ = self._display_res[0] + x * self._display_res[0] / 1920
            if y > 0:
                y_ = y * self._display_res[1] / 1080
            else:
                y_ = self._display_res[1] + y * self._display_res[1] / 1080
            return x_, y_

        self.labs["FPS"].setFont(QtGui.QFont("monospace", 12, 150))
        self.labs["FPS"].setColor("white")
        self.labs["FPS"].setPos(*rescale(20, 30))

        self.labs["CLAHE"].setFont(QtGui.QFont("monospace", 12, 150))
        self.labs["CLAHE"].setColor("white")
        self.labs["CLAHE"].setPos(*rescale(20, 105))

        self.labs["Gain"].setFont(QtGui.QFont("monospace", 12, 150))
        self.labs["Gain"].setColor("white")
        self.labs["Gain"].setPos(*rescale(20, 55))

        self.labs["Exp"].setFont(QtGui.QFont("monospace", 12, 150))
        self.labs["Exp"].setColor("white")
        self.labs["Exp"].setPos(*rescale(20, 80))

        self.labs["Saving"].setPos(*rescale(-250, -30))
        self.labs["Saving"].setFont(QtGui.QFont("monospace", 11, 700))
        self.labs["Saving"].setColor("r")

        self.labs["Save queue"].setPos(*rescale(-250, -55))
        self.labs["Save queue"].setFont(QtGui.QFont("monospace", 11, 700))
        self.labs["Save queue"].setColor("r")

        self.labs["Time"].setPos(*rescale(30, -30))
        self.labs["Time"].setFont(QtGui.QFont("monospace", 11, 100))
        self.labs["Time"].setColor("g")

        self.labs["GIM"].setPos(*rescale(-300, 30))
        self.labs["GIM"].setFont(QtGui.QFont("monospace", 11, 100))
        self.labs["GIM"].setColor("g")

        self.labs["IMU"].setPos(*rescale(-300, 55))
        self.labs["IMU"].setFont(QtGui.QFont("monospace", 11, 700))
        self.labs["IMU"].setColor("g")

        self.labs["TrackAlt"].setPos(*rescale(-300, 90))
        self.labs["TrackAlt"].setFont(QtGui.QFont("monospace", 11, 150))
        self.labs["TrackAlt"].setColor("g")

        self.crosshairs()

        self.app.processEvents()
        # self.win.showMaximized()
        self.win.setContentsMargins(0, 0, 0, 0)
        self.win.show()

    def set_clahe(self, enabled: bool, clip_limit: float = 2.0):
        with self._stream_lock:
            if enabled:
                self._disp_set.clahe_enabled = True
                self._disp_set.clahe = cv2.createCLAHE(
                    clipLimit=clip_limit, tileGridSize=(16, 16)
                )
            else:
                self._disp_set.clahe_enabled = False
                self._disp_set.clahe = None

    def set_norm(self, enabled: bool):
        with self._stream_lock:
            if enabled:
                self._disp_set.norm_enabled = True
            else:
                self._disp_set.norm_enabled = False

    def set_colourmap(self, enabled: bool):
        with self._stream_lock:
            if enabled:
                self._disp_set.colourmap_enabled = True
            else:
                self._disp_set.colourmap_enabled = False

    @staticmethod
    def soft_normalise(img, lower=0, upper=100):
        low_val = np.percentile(img, lower)
        high_val = np.percentile(img, upper)
        img_clipped = np.clip(img, low_val, high_val)
        norm_img = ((img_clipped - low_val) / (high_val - low_val) * 255).astype(
            np.uint8
        )
        return norm_img

    def update_img(self, img):
        """Update the image displayed in the window.
        Note: This img is assumed to be 8-bit grayscale.
        Args:
            img (img): The img to display.
        """

        # img = img.astype(np.uint8)
        # img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        if self._disp_set.norm_enabled:
            img = self.soft_normalise(img)

        if self._disp_set.clahe_enabled:
            try:
                img = self._disp_set.clahe.apply(img)
            except:
                warnings.warn("Can't apply CLAHE filter")
                self.set_clahe(False)

        if self._disp_set.colourmap_enabled:
            local_mean = cv2.GaussianBlur(img, (25, 25), 5)
            contrast = cv2.subtract(img, local_mean)
            normalised = self.soft_normalise(contrast, 1, 99)
            img = cv2.applyColorMap(normalised, cv2.COLORMAP_INFERNO)
        self.img.setImage(img, axis=0, levels=[0, 255])

    def update_labels(self, lab_data):
        for key, item in lab_data.items():
            self.labs[key].setText(key + ": " + str(item))

    def update_tracking(self, timestamp, hpr_euler):
        uv_path, uv_pt = self._overlay.project_from_ned_angles(hpr_euler, timestamp)
        grad = np.gradient(uv_path[:,1], uv_path[:,0])
        theta = np.arctan(grad)
        upper_bound = np.array([uv_path[:,0] + np.sin(theta) * self.offset, uv_path[:,1] - np.cos(theta) * self.offset])
        lower_bound = np.array([uv_path[:,0] - np.sin(theta) * self.offset, uv_path[:,1] + np.cos(theta) * self.offset])
        self.traj_bounds_upper.setData(
            y=self._display_res[1] - upper_bound[1],
            x=1 * upper_bound[0],
            pen="b",
        )
        self.traj_bounds_lower.setData(
            y=self._display_res[1] - lower_bound[1],
            x=1 * lower_bound[0],
            pen="b",
        )
        self.traj_time_indicator.setData(
            y=[
                self._display_res[1] - uv_pt[0,1] + self.offset,
                self._display_res[1] - uv_pt[0,1] + 2 * self.offset,
                self._display_res[1] - uv_pt[0,1] - self.offset,
                self._display_res[1] - uv_pt[0,1] - 2* self.offset
                ],
            x=[
                uv_pt[0, 0] - self.offset,
                uv_pt[0, 0] - 2 * self.offset,
                uv_pt[0, 0] + self.offset,
                uv_pt[0, 0] + 2 * self.offset,
            ],
            connect="pairs",
            pen="r",
        )

    def crosshairs(self):

        self.x_rules.setData(
            y=[
                1 * self._display_res[1] / 8,
                3 * self._display_res[1] / 8,
                5 * self._display_res[1] / 8,
                7 * self._display_res[1] / 8,
            ],
            x=[
                self._display_res[0] / 2,
                self._display_res[0] / 2,
                self._display_res[0] / 2,
                self._display_res[0] / 2,
            ],
            connect="pairs",
            pen="g",
        )
        self.y_rules.setData(
            y=[
                self._display_res[1] / 2,
                self._display_res[1] / 2,
                self._display_res[1] / 2,
                self._display_res[1] / 2,
            ],
            x=[
                1 * self._display_res[0] / 8,
                3 * self._display_res[0] / 8,
                5 * self._display_res[0] / 8,
                7 * self._display_res[0] / 8,
            ],
            connect="pairs",
            pen="g",
        )

    def run(self):
        with self._stream, self._imu_monitor, self._encoder_monitor:
            while not self._kill_event.is_set():
                frame = self._stream.latest_frame()
                if frame is not None:
                    frame = self._stream.cam.convert_for_monitoring(frame)
                    lab_data = {
                        "Exp": frame.exposure,
                        "Gain": frame.gain,
                        "FPS": np.round(self._stream.cam.frame_rate, 2),
                        "CLAHE": "Enabled" if self._disp_set.clahe_enabled else "",
                        "Saving": (
                            "Enabled" if self._stream.save_enabled else "Disabled"
                        ),
                        "Save queue": self._stream.save_queue_length,
                        "Time": datetime.datetime.fromtimestamp(
                            frame.timestamp, tz=datetime.timezone.utc
                        ).strftime("%H:%M:%S.%f")[:-5],
                    }
                    if self._has_imu:
                        # try:
                        if True:
                            euler = self._imu_monitor.get_hpr(frame.timestamp)
                            lab_data["IMU"] = (
                                f"Head {euler[0]:.2f} Pitch {euler[1]:.2f}"
                            )
                            if self._has_overlay:
                                self.update_tracking(frame.timestamp, euler)
                        # except Exception as e:
                        #     print(e)
                        #     pass
                    if self._has_encoder:
                        try:
                            azel = self._encoder_monitor.get_azel(frame.timestamp)
                            lab_data["GIM"] = f"Az {azel[0]:.2f}, El {azel[1]:.2f}"
                        except:
                            pass
                    img = self.downscale(frame.pixels)
                    self.update_img(img)
                    self.update_labels(lab_data)
                        # self.update_tracking(frame.timestamp)
                    self.app.processEvents()
            self.p1.close()
            self.win.close()
            self.app.closeAllWindows()

    def close(self):
        """Close the display window."""
        self._kill_event.set()
