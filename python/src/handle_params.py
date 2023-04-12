import os.path
import toml
from numpy import pi
from SingleTrackModel import SingleTrackModel
from DoubleTrackModel import DoubleTrackModel
import sys
import shutil

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 27.01.2021
"""


def load(plot_config_file, album_config_file, sim_config_file, veh_config_file, folder):
    """
    Documentation
    Loads parameter from toml files

    Input:
    plot_config_file:       name of the file as string
    album_config_file:      name of the file as string
    sim_config_file:        name of the file as string
    veh_config_file:        name of the file as string

    Output:
    plot_config:            dictionary containing the plot config parameter
    album_config:           dictionary containing the album config parameter
    sim_config:             dictionary containing the sim config parameter
    veh_config:             dictionary containing the veh config parameter
    """

    # add .toml
    plot_config_file = plot_config_file + ".toml"
    album_config_file = album_config_file + ".toml"
    sim_config_file = sim_config_file + ".toml"
    veh_config_file = veh_config_file + ".toml"

    # assemble path to param files
    path_root2module = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path_params = os.path.join(path_root2module, 'params')
    path_params_copy = os.path.join(folder, 'params')

    # read plot config file
    try:
        with open(os.path.join(path_params, plot_config_file), 'r') as fh:
            plot_config = toml.load(fh)
            shutil.copy(os.path.join(path_params, plot_config_file), path_params_copy)
    except FileNotFoundError:
        raise FileNotFoundError('plot config file does not exist or invalid target specifier "'
                                + plot_config_file + '" provided!') from None

    plot_config["delta_steps"] = plot_config["delta_steps"] * 2 + 1  # mirror delta steps into negative and add 1 for delta=0
    plot_config["beta_steps"] = plot_config["beta_steps"] * 2 + 1  # mirror beta steps into negative and add 1 for beta=0


    # read album config file
    try:
        with open(os.path.join(path_params, album_config_file), 'r') as fh:
            album_config = toml.load(fh)
            shutil.copy(os.path.join(path_params, album_config_file), path_params_copy)
    except FileNotFoundError:
        raise FileNotFoundError('album config file does not exist or invalid target specifier "'
                                + album_config_file + '" provided!') from None


    # read sim config file
    try:
        with open(os.path.join(path_params, sim_config_file), 'r') as fh:
            sim_config = toml.load(fh)
            shutil.copy(os.path.join(path_params, sim_config_file), path_params_copy)
    except FileNotFoundError:
        raise FileNotFoundError('sim config file does not exist or invalid target specifier "'
                                + sim_config_file + '" provided!') from None
    sim_config["banking"] = sim_config["banking"]/180*pi


    # read vehicle config file
    try:
        with open(os.path.join(path_params, veh_config_file), 'r') as fh:
            veh_config = toml.load(fh)
            shutil.copy(os.path.join(path_params, veh_config_file), path_params_copy)
    except FileNotFoundError:
        raise FileNotFoundError('vehicle config file does not exist or invalid target specifier "'
                                + veh_config_file + '" provided!') from None

    return plot_config, album_config, sim_config, veh_config

def veh(veh, veh_config):
    """
    Documentation
    Loads internal vehicle parameter contained in the TUM vehicle dynamics package

    Input:
    veh:                    TUM_vehicle_dynamics package
    veh_config:             Dictionary containing the loaded vehicle parameter

    Output:
    veh_config:             Dictionary containing the loaded vehicle parameter
    """
    l_f = veh.get_parameter("VEH__VehicleData__l_WheelbaseF_m")
    l_r = veh.get_parameter("VEH__VehicleData__l_WheelbaseTotal_m") - l_f
    tw_r = veh.get_parameter("VEH__VehicleData__w_TrackR_m")
    if veh.get_parameter("tireFL__MFSIMPLE__r_tire_m") > 0:
        r_front = veh.get_parameter("tireFL__MFSIMPLE__r_tire_m")
    else:
        r_front = veh.get_parameter("tireFL__WHEEL__TYRE_RADIUS_MOD")

    if veh.get_parameter("tireRL__MFSIMPLE__r_tire_m") > 0:
        r_rear = veh.get_parameter("tireRL__MFSIMPLE__r_tire_m")
    else:
        r_rear = veh.get_parameter("tireRL__WHEEL__TYRE_RADIUS_MOD")

    veh_config.update({"l_front": l_f, "l_rear": l_r, "tw_r": tw_r, "r_front": r_front, "r_rear": r_rear})
    return veh_config

def model_select(sim_config):
    if sim_config["model"] == 1:
        veh=DoubleTrackModel()
    elif sim_config["model"] == 2:
        veh=SingleTrackModel()
    else:
        sys.exit('Check sim_config__file: invalid index given for "model"')
    return veh

