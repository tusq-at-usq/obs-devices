import threading
import time
import matplotlib.pyplot as plt

from obs_utils.context import State


class StatePlotter(threading.Thread):
    def __init__(self, state, interval=0.2):
        super().__init__(daemon=True)
        self.state = state  # system.state is being updated elsewhere
        self.interval = interval  # seconds
        self._running = True

        data_labels = [
            "head_imu",
            "pitch_imu",
            "head_target",
            "pitch_target",
            "u_az, u_el",
            "int_e_az",
            "int_e_el",
            "gim_az",
            "gim_el",
        ]
        self.data = {label: [] for label in data_labels}

        # storage for plotting
        self.time_target = []
        self.time_imu =[]
        self.time_gimbal = []

    def add_data(self):
        state = self.state
        self.data["head_imu"].append(float(state.imu_state.heading))
        self.data["pitch_imu"].append(float(state.imu_state.pitch))
        self.data["head_target"].append(float(state.gimbal_state.sp_az))
        self.data["pitch_target"].append(float(state.gimbal_state.sp_el))
        self.data["u_az"].append(float(state.gimbal_state.u_az))
        self.data["u_el"].append(float(state.gimbal_state.u_el))
        self.data["int_e_az"].append(float(state.gimbal_state.int_err_az))
        self.data["int_e_el"].append(float(state.gimbal_state.int_err_el))
        self.data["gim_az"].append(float(state.az))
        self.data["gim_el"].append(float(state.el))
        self.time_imu.append(float(state.imu_state.t_pc))
        self.time_target.append(float(state.gimbal_state.t_ctrl_update))
        self.time_gimbal.append(float(state.gimbal_state.t_pos))

    def stop(self):
        self._running = False

    def run(self):
        plt.ion()
        fig, ax = plt.subplots(2,1, sharex=True)
        (az_line_target,) = ax[0].plot([], [], label="Target")
        (az_line_imu,) = ax[0].plot([], [], label="IMU")
        (az_line_gimbal,) = ax[0].plot([], [], label="Gimb")
        
        (el_line_target,) = ax[1].plot([], [], label="Target")
        (el_line_imu,) = ax[1].plot([], [], label="IMU")
        (el_line_gimbal,) = ax[1].plot([], [], label="Gimb")
        ax[0].legend()

        ax[1].set_xlabel("Time (s)")

        while self._running:
            time.sleep(self.interval)

            self.add_data()
            # update plot
            az_line_target.set_data(self.time_target[-60:0], self.data["head_target"][-60:0])
            az_line_imu.set_data(self.time_imu[-60:0], self.data["head_imu"][-60:0])
            az_line_gimbal.set_data(self.time_gimbal[-60:0], self.data["gim_az"][-60:0])
            el_line_target.set_data(self.time_target[-60:0], self.data["pitch_target"][-60:0])
            el_line_imu.set_data(self.time_imu[-60:0], self.data["pitch_imu"][-60:0])
            el_line_gimbal.set_data(self.time_gimbal[-60:0], self.data["gim_el"][-60:0])
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()

        plt.ioff()
        plt.show()
