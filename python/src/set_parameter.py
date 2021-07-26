import numpy as np
import sys
import os

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""

def set_parameter(veh, v, v_on, veh_config, sim_config):
    """
    Documentation
    alters vehicle parameter inside the TUM_vehicle_dynamics module

    Input:
    veh:                    TUM_vehicle_dynamics package
    v:                      set longitudinal velocity [m/s]
    sim_config:             dictionary containing the sim config parameter
    veh_config:             dictionary containing the veh config parameter

    Output:
    """
    if v_on == 1:
        # set initial velocity and wheel rotation
        veh.set_parameter("INIT__initialvalues__Vehicle_InitialVelocity_mps__1", v)

        u_front = veh_config["r_front"]*2*np.pi
        u_rear = veh_config["r_rear"]*2*np.pi
        omega_front=(v/u_front)*2*np.pi
        omega_rear=(v/u_rear)*2*np.pi
        veh.set_parameter("INIT__initialvalues__omega0_Wheels_radps__1", omega_front)
        veh.set_parameter("INIT__initialvalues__omega0_Wheels_radps__2", omega_front)
        veh.set_parameter("INIT__initialvalues__omega0_Wheels_radps__3", omega_rear)
        veh.set_parameter("INIT__initialvalues__omega0_Wheels_radps__4", omega_rear)

    # set tire scaling parameters
    if sim_config["model"] == 1:
        muex = np.ones(4) * veh_config["muex"]
        muey = np.ones(4) * veh_config["muey"]
        veh.set_lambdaMuex(muex)
        veh.set_lambdaMuey(muey)
    if sim_config["model"] == 2:
        muexf = veh.get_parameter("tireFL__MFSIMPLE__PacLong_D") * veh_config["muex"]
        mueyf = veh.get_parameter("tireFL__MFSIMPLE__PacLat_D") * veh_config["muey"]
        muexr = veh.get_parameter("tireRL__MFSIMPLE__PacLong_D") * veh_config["muex"]
        mueyr = veh.get_parameter("tireRL__MFSIMPLE__PacLat_D") * veh_config["muey"]
        veh.set_parameter("tireFL__MFSIMPLE__PacLong_D", muexf)
        veh.set_parameter("tireFL__MFSIMPLE__PacLat_D", mueyf)
        veh.set_parameter("tireRL__MFSIMPLE__PacLong_D", muexr)
        veh.set_parameter("tireRL__MFSIMPLE__PacLat_D", mueyr)

    #set additional vehicle parameter
    if len(veh_config["VEH"]) != 0:
        for key, value in veh_config["VEH"].items():
            if key.startswith("variable"):
                name = value
            elif key.startswith("value"):
                param = value
                veh.set_parameter(name, param)
            else:
                sys.exit('Check veh_config_file section [VEH]. There might be issue with parameter order or naming.')
    else:
        print('baseline vehicle parameter loaded')

def set_fx(veh, ax_set):
    mass_veh = veh.get_parameter("VEH__VehicleData__m_Vehicle_kg")
    fx_ext = -mass_veh * ax_set
    fx_ext_arr = np.zeros(3)
    fx_ext_arr[0] = fx_ext

    return fx_ext_arr, mass_veh

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__