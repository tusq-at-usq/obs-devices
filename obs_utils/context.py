from __future__ import annotations
from contextlib import ExitStack
import threading
from functools import cached_property

from obs_controller.controller import GimbalController, GimbalState
from obs_cameras.base import CameraStream
from obs_certus.monitor import CertusMonitor, IMUState, imu_extrapolate_full
from obs_encoders.monitor import EncoderMonitor, EncoderState

# TODO: Put some check for stale data and return none if too old?

class State:
    _imu_state: IMUState | None
    _encoder_state: EncoderState | None
    _gimbal_state: GimbalState | None
    _lock: threading.RLock

    def __init__(
        self,
        imu_state: IMUState | None = None,
        encoder_state: EncoderState | None = None,
        gimbal_state: GimbalState | None = None,
    ):
        self._imu_state = imu_state
        self._encoder_state = encoder_state
        self._gimbal_state = gimbal_state
        self._lock = threading.RLock()

    @property
    def imu_state(self) -> IMUState | None:
        with self._lock:
            return self._imu_state

    def set_imu_state(self, value: IMUState | None) -> None:
        with self._lock:
            self._imu_state = value

    @property
    def gimbal_state(self) -> GimbalState | None:
        with self._lock:
            return self._gimbal_state

    def set_gimbal_state(self, value: GimbalState | None) -> None:
        with self._lock:
            self._gimbal_state = value

    @property
    def encoder_state(self) -> EncoderState | None:
        with self._lock:
            return self._encoder_state

    def set_encoder_state(self, value: EncoderState | None) -> None:
        with self._lock:
            self._encoder_state = value

    def extrap_imu_state(self, t_unix: float, pc_time: bool = False) -> IMUState:
        if self._imu_state is None:
            raise RuntimeError("No IMU state available to extrapolate from")
        with self._lock:
            return imu_extrapolate_full(
                self._imu_state, t_unix, pc_time=pc_time
            )


class Context:
    _controller: GimbalController | None
    _streams: list[CameraStream]
    _imu_monitor: CertusMonitor | None
    _enc_monitor: EncoderMonitor | None
    _stack: ExitStack
    _disp_stream_idx: int
    _lock: threading.RLock

    def __init__(
        self,
        controller: GimbalController | None = None,
        streams: list[CameraStream] = [],
        imu_monitor: CertusMonitor | None = None,
        enc_monitor: EncoderMonitor | None = None,
    ):
        self._controller = controller
        self._streams = streams
        self._imu_monitor = imu_monitor
        self._enc_monitor = enc_monitor
        self._disp_stream_idx = 0
        self._lock = threading.RLock()

        self._stack = ExitStack()

    def __enter__(self) -> Context:
        self._stack.__enter__()

        self._streams = [self._stack.enter_context(stream) for stream in self._streams]
        if self._controller is not None:
            self._controller = self._stack.enter_context(self._controller)
        if self._imu_monitor is not None:
            self._imu_monitor = self._stack.enter_context(self._imu_monitor)
        if self._enc_monitor is not None:
            self._enc_monitor = self._stack.enter_context(self._enc_monitor)
        return self

    @cached_property
    def has_controller(self) -> bool:
        return self._controller is not None

    @cached_property
    def has_imu_monitor(self) -> bool:
        return self._imu_monitor is not None

    @cached_property
    def has_enc_monitor(self) -> bool:
        return self._enc_monitor is not None

    @property
    def controller(self) -> GimbalController:
        if self._controller is None:
            raise RuntimeError("No controller available in this context")
        return self._controller

    @property
    def imu_monitor(self) -> CertusMonitor:
        if self._imu_monitor is None:
            raise RuntimeError("No IMU monitor available in this context")
        return self._imu_monitor

    @property
    def enc_monitor(self) -> EncoderMonitor:
        if self._enc_monitor is None:
            raise RuntimeError("No position monitor available in this context")
        return self._enc_monitor

    @property
    def disp_stream(self) -> CameraStream:
        if len(self._streams) == 0:
            raise RuntimeError("No camera streams available in this context")
        with self._lock:
            return self._streams[self._disp_stream_idx]

    def change_display_stream(self, i: int) -> None:
        with self._lock:
            if i < 0 or i >= len(self._streams):
                raise ValueError("Invalid stream index")
            self._disp_stream_idx = i

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self._stack.__exit__(exc_type, exc_value, traceback)

    def save_all(self) -> None:
        for stream in self._streams:
            stream.save_enabled = True

    def stop_saving_all(self) -> None:
        for stream in self._streams:
            stream.save_enabled = False
