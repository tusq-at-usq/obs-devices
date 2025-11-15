import threading
import time
import matplotlib.pyplot as plt

from obs_utils.context import State


class StatePlotter:
    def __init__(self, state, interval=0.1):
        self.state = state  # system.state is being updated elsewhere
        self.interval = interval  # seconds
        self._running = True

        data_labels = [
            "head_imu",
            "pitch_imu",
            "head_target",
            "pitch_target",
            "u_az",
            "u_el",
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
        self.time_imu.append(float(state.imu_state.t_pc))
        if state.gimbal_state is not None:
            self.data["head_target"].append(float(state.gimbal_state.sp_az))
            self.data["pitch_target"].append(float(state.gimbal_state.sp_el))
            self.data["u_az"].append(float(state.gimbal_state.u_az))
            self.data["u_el"].append(float(state.gimbal_state.u_el))
            self.data["int_e_az"].append(float(state.gimbal_state.int_err_az))
            self.data["int_e_el"].append(float(state.gimbal_state.int_err_el))
            self.data["gim_az"].append(float(state.gimbal_state.az))
            self.data["gim_el"].append(float(state.gimbal_state.el))
            self.time_target.append(float(state.gimbal_state.t_ctrl_update))
            self.time_gimbal.append(float(state.gimbal_state.t_pos))

        for key in self.data.keys():
            self.data[key] = self.data[key][-60:]
            self.time_imu = self.time_imu[-60:]
            self.time_target = self.time_target[-60:]
            self.time_gimbal = self.time_gimbal[-60:]


    def stop(self):
        self._running = False

    def run(self):
        plt.ion()
        fig, ax = plt.subplots(2,1, sharex=True)
        (az_line_target,) = ax[0].plot([], [], label="Target")
        (az_line_imu,) = ax[0].plot([], [], label="IMU")
        (az_line_gimbal,) = ax[0].plot([], [], label="Gimb")
        (az_line_u,) = ax[0].plot([], [], label="u")
        
        
        (el_line_target,) = ax[1].plot([], [], label="Target")
        (el_line_imu,) = ax[1].plot([], [], label="IMU")
        (el_line_gimbal,) = ax[1].plot([], [], label="Gimb")
        (el_line_u,) = ax[1].plot([], [], label="u")
        ax[0].legend()

        ax[1].set_xlabel("Time (s)")

        while self._running:
            time.sleep(self.interval)

            self.add_data()
            # print(self.time_target)
            # print(self.data["head_target"])
            # update plot
            az_line_target.set_data(self.time_target, self.data["head_target"])
            az_line_imu.set_data(self.time_imu, self.data["head_imu"])
            az_line_gimbal.set_data(self.time_gimbal, self.data["gim_az"])
            az_line_u.set_data(self.time_gimbal, self.data["u_az"])
            el_line_target.set_data(self.time_target, self.data["pitch_target"])
            el_line_imu.set_data(self.time_imu, self.data["pitch_imu"])
            el_line_gimbal.set_data(self.time_gimbal, self.data["gim_el"])
            el_line_u.set_data(self.time_gimbal, self.data["u_el"])
            for a in ax:
                a.relim()
                a.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()

        plt.ioff()
        plt.show()
