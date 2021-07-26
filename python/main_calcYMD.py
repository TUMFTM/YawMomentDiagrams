import numpy as np
import src
import os
import datetime
from statistics import *
from scipy.io import savemat
import matplotlib.pyplot as plt
import time
import concurrent.futures
from math import atan, cos, sin

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021

Documentation:
Main script to generate yaw moment diagrams (and further derived plots and KPI) via the Milliken Moment Method

Requirements:
1. All Packages of the Requirements.txt need to be installed
2. The TUM vehicle dynamics Package must be installed
"""

# # -------------------------------------------------------------------------------------------------------------------
# USER INPUT ----------------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------

# assign valid param files
plot_config_file = 'plot_default'
album_config_file = 'album_default'
sim_config_file = 'sim_DTM_default'
veh_config_file = 'veh_default'

# # -------------------------------------------------------------------------------------------------------------------
# # Initialization ---------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------

# create directories
folder = os.path.join(os.getcwd(), 'output', datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
src.directories.create(folder)

# load parameter
plot_config, album_config, sim_config, veh_config = src.handle_params.load(plot_config_file, album_config_file,
                                                                           sim_config_file, veh_config_file, folder)
veh = src.handle_params.model_select(sim_config)
src.set_parameter.blockPrint()
veh_config = src.handle_params.veh(veh, veh_config)
src.set_parameter.enablePrint()
src.set_parameter.set_parameter(veh, 0, 0, veh_config, sim_config)
time.sleep(3)
print("Let's go!")
time.sleep(2)

# initialize variables
n_sim = sim_config["n_sim"]
n_eval = sim_config["n_eval"]
ay_mps2 = np.zeros(n_sim)
sa_f = np.zeros(n_sim)
sa_r = np.zeros(n_sim)
dPsi = np.zeros(n_sim)
DriveTorque_arr = np.zeros(4)
BrakePressure_arr = np.zeros(4)
extYMz_arr = np.zeros((n_sim, 3))
time_vec = [0.002 * t for t in range(0, n_sim)]
ai = album_config["ax_steps"]
vi = album_config["v_steps"]
bi = plot_config["beta_steps"]
di = plot_config["delta_steps"]
ay_table = np.zeros((ai, vi, bi, di), dtype='float32')
ym_table = np.zeros((ai, vi, bi, di), dtype='float32')
sa_f_table = np.zeros((ai, vi, bi, di), dtype='float32')
sa_r_table = np.zeros((ai, vi, bi, di), dtype='float32')
dpsi_table = np.zeros((ai, vi, bi, di), dtype='float32')
ay_std_table = np.zeros((ai, vi, bi, di), dtype='float32')

# start time counter
t_start = time.perf_counter()

# # -------------------------------------------------------------------------------------------------------------------
# # Simulation----- ---------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------


# simulate model with constant steering angle and force request
def simulate(ax_set, vx_set, beta_set, delta_set):
    # initialize vehicle
    src.set_parameter.blockPrint()
    veh = src.handle_params.model_select(sim_config)
    src.set_parameter.set_parameter(veh, vx_set, 1, veh_config, sim_config)
    f_ext, mass_veh = src.set_parameter.set_fx(veh, ax_set)
    delta = 0
    beta = 0
    if sim_config["model"] == 1:
        beta_set = -beta_set
    veh.set_SteeringAngle(delta)
    veh.set_externalForce(f_ext)
    src.set_parameter.enablePrint()

    # initialize controller
    velocity_PID = src.PID.PID(sim_config["velPIDparams"])
    beta_PID = src.PID.PID(sim_config["ymPIDparams"])
    brake_PID = src.PID.PID(sim_config["brakePIDparams"])

    for i in range(n_sim):

        # get current vehicle dynamic state
        vx_mps = veh.get_vx_mps()
        vy_mps = veh.get_vy_mps()
        ay_mps2[i] = veh.get_ay_mps2()
        dpsi_radps = veh.get_dPsi_radps()

        # calculate controller outputs
        delta = delta + 0.002 * (delta_set - delta)
        veh.set_SteeringAngle(delta)
        if f_ext[0] <= 0:
            DriveTorque = velocity_PID.run(vx_set, vx_mps)
            if veh_config["tv"] > 0:
                yawmoment = ay_mps2[i] * veh_config["tv"]
                single_wheel_force = yawmoment / (veh_config["tw_r"] / 2)
                single_wheel_torque = single_wheel_force * veh_config["r_rear"]
                two_wheel_torque = single_wheel_torque / 2
                DriveTorque_arr[2] = 0.5 * DriveTorque - two_wheel_torque
                DriveTorque_arr[3] = 0.5 * DriveTorque + two_wheel_torque
            else:
                DriveTorque_arr[2] = 0.5 * DriveTorque
                DriveTorque_arr[3] = 0.5 * DriveTorque
            veh.set_DriveTorque(DriveTorque_arr)
        if f_ext[0] > 0:
            BrakePressure = -brake_PID.run(vx_set, vx_mps)
            p_front = veh_config["brake_balance"] * BrakePressure
            p_rear = (1 - veh_config["brake_balance"]) * BrakePressure
            BrakePressure_arr[0] = p_front
            BrakePressure_arr[1] = p_front
            BrakePressure_arr[2] = p_rear
            BrakePressure_arr[3] = p_rear
            veh.set_BrakePressure(BrakePressure_arr)
        if vx_mps != 0:
            beta_act = atan(vy_mps / vx_mps)
        else:
            beta_act = 0
        if sim_config["model"] == 2:
            beta_act = - beta_act
        beta = beta + 0.002 * (beta_set - beta)
        extYMz = -beta_PID.run(beta, -beta_act)
        extYMz_arr[i, 2] = extYMz
        veh.set_externalTorque(extYMz_arr[i])

        # calculate banking
        if sim_config["banking"] != 0:
            ay = ay_mps2[i]
            banking = sim_config["banking"]
            g = 9.81
            fy_ext = (1 - cos(banking)) * mass_veh * (-ay) + sin(-banking) * mass_veh * g
            fz_ext = sin(-banking) * mass_veh * ay + (1 - cos(banking)) * (-mass_veh) * g
            f_ext[1] = fy_ext
            f_ext[2] = fz_ext
            veh.set_externalForce(f_ext)

        # calculate slip angle front and rear and log yaw rate
        if i > (n_sim - n_eval):
            sa_f[i] = delta_set - beta_act - (veh_config["l_front"] / vx_mps) * dpsi_radps
            sa_r[i] = -beta_act + (veh_config["l_rear"] / vx_mps) * dpsi_radps
            dPsi[i] = dpsi_radps

        # simulate
        veh.step()

    del veh
    print(f'ax:{int(ax_set)}; vx:{int(vx_set)}; beta:{beta_set.round(3)}; delta:{delta_set.round(3)} done')
    return mean(ay_mps2[(n_sim - n_eval):n_sim]), mean(extYMz_arr[(n_sim - n_eval):n_sim, 2]), \
           mean(sa_f[(n_sim - n_eval):n_sim]), mean(sa_r[(n_sim - n_eval):n_sim]), mean(dPsi[(n_sim - n_eval):n_sim]), \
           stdev(ay_mps2[(n_sim - n_eval):n_sim])


for ia in range(album_config["ax_steps"]):
    # generate input arrays for pool executor
    ax_arr, vx_arr, beta_arr, delta_arr, ax_range, v_range, beta_range, delta_range = \
        src.array_handling.allocate(plot_config, album_config, ia)

    # simulate using all available processor cores via process pool executor
    with concurrent.futures.ProcessPoolExecutor() as executor:
        results = executor.map(simulate, ax_arr, vx_arr, beta_arr, delta_arr)

    # Reshaping Results
    ay_table[ia, :, :, :], ym_table[ia, :, :, :], sa_f_table[ia, :, :, :], sa_r_table[ia, :, :, :], \
    dpsi_table[ia, :, :, :], ay_std_table[ia, :, :, :] = \
        src.array_handling.results_arrays(results, album_config, plot_config, len(ax_arr), sim_config)

# Print running time
t_stop = time.perf_counter()
print("simulation time: " + str(t_stop - t_start) + "s")
print("... hang on just a sec: post processing...")
t_start = time.perf_counter()

# # -------------------------------------------------------------------------------------------------------------------
# # Post Processing ---------------------------------------------------------------------------------------------------
# # -------------------------------------------------------------------------------------------------------------------

# Allocate Variables for Post Processing
sz = ay_table.shape
clean_table = np.zeros((sz[0], sz[1], sz[2], sz[3]))
stability_table = np.zeros((sz[0], sz[1], sz[2], sz[3]))
control_table = np.zeros((sz[0], sz[1], sz[2], sz[3]))
steering_sens_table = np.zeros((sz[0], sz[1], sz[2], sz[3]))
sa_diff_table = np.zeros((sz[0], sz[1], sz[2], sz[3]))
kpi = np.zeros((sz[0], sz[1], 2, 19))

# save output file
name = '/output.mat'
saveas = folder + name
savemat(saveas, {"ax_range": ax_range, "v_range": v_range, "beta_range": beta_range,"ay_std_table": ay_std_table,
                 "delta_range": delta_range, "ay_table": ay_table, "ym_table": ym_table, "dpsi_table": dpsi_table})

# Generate Plots and KPI
for ia in range(ay_table.shape[0]):
    for iv in range(ay_table.shape[1]):

        # calculate clean table
        clean_table[ia, iv, :, :] = src.plot.clean_table(ay_table, ia, iv)

        # generate YMD plots
        if iv == (sz[1] - 1):
            fig = src.plot.YMDplotly(ay_table, ym_table, clean_table, ax_range, v_range, beta_range, delta_range,
                                     plot_config, ia)
            name = f'/YMD_ax{int(ax_range[ia])}mps2'
            saveas = folder + name
            fig.write_html(saveas)

        # KPI calculation
        stability_table[ia, iv, :, :], control_table[ia, iv, :, :], steering_sens_table[ia, iv, :, :], \
        sa_diff_table[ia, iv, :, :] = src.calc_KPI.matrices(ay_table, ym_table, beta_range, delta_range, sa_f_table,
                                                            sa_r_table, ia, iv)

        kpi[ia, iv, :, :] = src.calc_KPI.calc(ay_table, ym_table, ia, iv, sa_f_table, sa_r_table, beta_range,
                                              delta_range,
                                              stability_table, control_table, steering_sens_table, clean_table)

        # stability plot
        fig, cb = src.plot.contour(ay_table, ym_table, stability_table, clean_table, plot_config, ia, iv)
        plt.title(f'stability @ {v_range[iv]}m/s, {ax_range[ia]}m/s²')
        name = f'/stability/stability_{v_range[iv]}mps_ax{int(ax_range[ia])}mps2'
        cb.set_label('yaw moment per vehicle slip angle [Nm/deg]')
        saveas = folder + name
        plt.savefig(saveas, dpi=plot_config["dpi"], format=plot_config["frmt"])
        plt.close(fig='all')

        # slip angle delta plot
        fig, cb = src.plot.contour(ay_table, ym_table, sa_diff_table, clean_table, plot_config, ia, iv)
        plt.title(f'slip angle difference @ {v_range[iv]}m/s, {ax_range[ia]}m/s²')
        name = f'/slip_angle_diff/slip_angle_diff_{v_range[iv]}mps_ax{int(ax_range[ia])}mps2'
        cb.set_label('slip angle difference [deg]')
        saveas = folder + name
        plt.savefig(saveas, dpi=plot_config["dpi"], format=plot_config["frmt"])
        plt.close(fig='all')

        # control moment plot
        fig, cb = src.plot.contour2(ay_table, ym_table, control_table, clean_table, plot_config, ia, iv)
        plt.title(f'control moment @ {v_range[iv]}m/s, {ax_range[ia]}m/s²')
        cb.set_label('yaw moment per steering angle [Nm/deg]')
        name = f'/control_moment/control_moment_{v_range[iv]}mps_ax{int(ax_range[ia])}mps2'
        saveas = folder + name
        plt.savefig(saveas, dpi=plot_config["dpi"], format=plot_config["frmt"])
        plt.close(fig='all')

        # control force plot
        fig, cb = src.plot.contour2(ay_table, ym_table, steering_sens_table, clean_table, plot_config, ia, iv)
        plt.title(f'control force @ {v_range[iv]}m/s, {ax_range[ia]}m/s²')
        cb.set_label('lateral acceleration per steering angle [m/s²/deg]')
        name = f'/control_force/control_force_{v_range[iv]}mps_ax{int(ax_range[ia])}mps2'
        saveas = folder + name
        plt.savefig(saveas, dpi=plot_config["dpi"], format=plot_config["frmt"])
        plt.close(fig='all')
    print("...")

# save output file
name = '/output.mat'
saveas = folder + name
savemat(saveas, {"kpi": kpi, "ax_range": ax_range, "v_range": v_range, "beta_range": beta_range, "ay_std_table": ay_std_table,
                 "delta_range": delta_range, "ay_table": ay_table, "ym_table": ym_table, "dpsi_table": dpsi_table,
                 "stability_table": stability_table, "control_table": control_table, "steering_sens_table": steering_sens_table})

# Print running time
t_stop = time.perf_counter()
print("post processing time: " + str(t_stop - t_start) + "s")
