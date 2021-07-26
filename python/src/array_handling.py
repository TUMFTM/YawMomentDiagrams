import numpy as np

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""


def results_arrays(results, album_config, plot_config, length, sim_config):
    """
    Documentation
    This function reformats the result files via a 1D vector into an easy to index ndarray

    Input:
    results:       generator object containing the results of the simulation
    album_config:  dictionary containing the configuration regarding album layout
    plot_config:   dictionary containing the configuration regarding plot layout
    length:        length of the result vector
    sim_config:    dictionary of simulation config parameters

    Output
    ay_table:       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table        ndarray containing the converged yaw moments of the sim [Nm]
    sa_f_table      ndarray containing the calculated slip angle at the front axle [rad]
    sa_r_table      ndarray containing the calculated slip angle at the front axle [rad]
    """

    # Allocate 1D result arrays
    result_arr_ay = np.zeros(length)
    result_arr_ym = np.zeros(length)
    result_arr_sa_f = np.zeros(length)
    result_arr_sa_r = np.zeros(length)
    result_arr_dpsi = np.zeros(length)
    result_arr_ay_std = np.zeros(length)

    # Unpack results and write into 1D array
    i = 0
    for result in results:
        result_arr_ay[i] = result[0]
        if sim_config["model"] == 1:
            result_arr_ym[i] = result[1]
        elif sim_config["model"] == 2:
            result_arr_ym[i] = -result[1]
        result_arr_sa_f[i] = result[2]
        result_arr_sa_r[i] = result[3]
        result_arr_dpsi[i] = result[4]
        result_arr_ay_std[i] = result[5]
        i = i+1

    # convert 1D into nD arrays
    ay_table = result_arr_ay.reshape(album_config["v_steps"], plot_config["beta_steps"], plot_config["delta_steps"])
    ym_table = result_arr_ym.reshape(album_config["v_steps"], plot_config["beta_steps"], plot_config["delta_steps"])
    sa_f_table = result_arr_sa_f.reshape(album_config["v_steps"], plot_config["beta_steps"], plot_config["delta_steps"])
    sa_r_table = result_arr_sa_r.reshape(album_config["v_steps"], plot_config["beta_steps"], plot_config["delta_steps"])
    dpsi_table = result_arr_dpsi.reshape(album_config["v_steps"], plot_config["beta_steps"], plot_config["delta_steps"])
    ay_std_table = result_arr_ay_std.reshape(album_config["v_steps"], plot_config["beta_steps"], plot_config["delta_steps"])

    return ay_table, ym_table, sa_f_table, sa_r_table, dpsi_table, ay_std_table

def allocate(plot_config, album_config, ia):
    """
    Documentation
    This function generates the input vector for the simulation. Each of the 4 input values (ax, vx, beta, delta)
    will have a dedicated vector which will be passed to the simulation process pool executor

    Input:
    album_config:  dictionary containing the configuration regarding album layout
    plot_config:   dictionary containing the configuration regarding plot layout

    Output
    ax_arr:         input array for the simulation for the requested longitudinal acceleration [m/s²]
    vx_arr:         input array for the simulation for the requested longitudinal velocity [m/s]
    beta_arr:       input array for the simulation for the requested body slip angle [rad]
    delta_arr:      input array for the simulation for the requested steering angle (wheel) [rad]
    ax_range:       ax_arr without duplicates
    vx_range:       vx_arr without duplicates
    beta_range:     beta_arr without duplicates
    delta_range:    delta_arr without duplicates
    """

    delta_max = plot_config["delta_max"]
    delta_steps = plot_config["delta_steps"]
    beta_max = plot_config["beta_max"]
    beta_steps = plot_config["beta_steps"]
    ax_steps = album_config["ax_steps"]
    v_steps = album_config["v_steps"]
    delta_min = -delta_max  # mirror delta range into negative
    beta_min = -beta_max  # mirror beta range into negative
    ax_range = np.linspace(album_config["ax_min"], album_config["ax_max"], ax_steps)
    v_range = np.linspace(album_config["v_min"], album_config["v_max"], v_steps)
    delta_range = np.linspace(delta_min, delta_max, delta_steps,  dtype='float64')  # linear interpolation of delta range
    beta_range = np.linspace(beta_min, beta_max, beta_steps)  # linear interpolation of beta range
    nLines = len(v_range) * len(beta_range) * len(delta_range)
    nLines_b = len(beta_range) * len(delta_range)
    vx_arr = np.zeros(nLines, dtype='float64')
    delta_arr = np.zeros(nLines, dtype='float64')
    beta_arr = np.zeros(nLines, dtype='float64')
    beta_arr_small = np.zeros(nLines_b,  dtype='float64')

    ax_arr = np.ones(nLines, dtype='float64') * ax_range[ia]

    incv = round(nLines / len(v_range))
    for step_v in range(v_steps):
        bound_low_v = incv * step_v
        bound_high_v = incv * (step_v + 1)
        vx_arr[bound_low_v:bound_high_v] = v_range[step_v]

    incb = round(nLines / (len(v_range) * len(beta_range)))
    for step_b in range(beta_steps):
        bound_low_b = incb * step_b
        bound_high_b = incb * (step_b + 1)
        beta_arr_small[bound_low_b:bound_high_b] = beta_range[step_b]

    for concb in range(v_steps):
        bound_low_b = concb * beta_steps * delta_steps
        bound_high_b = (concb + 1) * beta_steps * delta_steps
        beta_arr[bound_low_b:bound_high_b] = beta_arr_small

    for concd in range(v_steps * beta_steps):
        bound_low_d = concd * delta_steps
        bound_high_d = (concd + 1) * delta_steps
        delta_arr[bound_low_d:bound_high_d] = delta_range

    return ax_arr, vx_arr, beta_arr, delta_arr, ax_range, v_range, beta_range, delta_range
