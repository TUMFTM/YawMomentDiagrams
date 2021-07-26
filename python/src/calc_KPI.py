import numpy as np
from math import nan
from math import pi

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""

def matrices(ay_table, ym_table, beta_range, delta_range, sa_f_table, sa_r_table, ia, iv):
    """
    Documentation
    This function reformats the result files via a 1D vector into an easy to index ndarray

    Input:
    ay_table:                       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table                        ndarray containing the converged yaw moments of the sim [Nm]
    beta_range:                     array with all beta values used in the simulation [rad]
    delta range:                    array with all delta values used in the simulation [rad]
    ia:                             iterator for long. acceleration index
    iv:                             iterator for long. velocity index

    Output
    stability_table:                ndarray with the stability kpi for each simulation scenario
    control_table:                  ndarray with the control kpi for each simulation scenario
    steering_sensitivity_table:     ndarray with the steering_sensitivity kpi for each simulation scenario
    """

    # get size of solutionmatrix
    sz = ay_table.shape
    beta_step = beta_range[1] - beta_range[0]
    delta_step = delta_range[1] - delta_range[0]

    # allocate variables
    stability1 = np.zeros((sz[2], sz[3]))
    stability2 = np.zeros((sz[2], sz[3]))
    control1 = np.zeros((sz[2], sz[3]))
    control2 = np.zeros((sz[2], sz[3]))
    steering_sens1 = np.zeros((sz[2], sz[3]))
    steering_sens2 = np.zeros((sz[2], sz[3]))

    # Calcutate stability matrix
    for ic in range(sz[3]):
        for ir in range(sz[2]):
            if ir == sz[2]-1:
                stability1[ir, ic] = nan
            else:
                stability1[ir, ic] = (ym_table[ia, iv, ir + 1, ic] - ym_table[ia, iv, ir, ic]) / beta_step / 180 * pi

            if ir == 0:
                stability2[ir, ic] = nan
            else:
                stability2[ir, ic] = (ym_table[ia, iv, ir, ic] - ym_table[ia, iv, ir - 1, ic]) / beta_step / 180 * pi


    stability_table = (stability1 + stability2) / 2

    # Calculate control and steering sensitivity matrix
    for ir in range(sz[2]):
        for ic in range(sz[3]):
            if ic == sz[3]-1:
                control1[ir, ic] = nan
                steering_sens1[ir, ic] = nan
            else:
                control1[ir, ic] = (ym_table[ia, iv, ir, ic + 1] - ym_table[ia, iv, ir, ic]) / delta_step / 180 * pi
                steering_sens1[ir, ic] = (ay_table[ia, iv, ir, ic + 1] - ay_table[ia, iv, ir, ic]) / delta_step / 180 * pi

            if ic == 0:
                control2[ir, ic] = nan
                steering_sens2[ir, ic] = nan
            else:
                control2[ir, ic] = (ym_table[ia, iv, ir, ic] - ym_table[ia, iv, ir, ic - 1]) / delta_step / 180 * pi
                steering_sens2[ir, ic] = (ay_table[ia, iv, ir, ic] - ay_table[ia, iv, ir, ic - 1]) / delta_step / 180 * pi

    control_table= (control1 + control2) / 2
    steering_sens_table = (steering_sens1 + steering_sens2) / 2

    sa_diff_table = (sa_f_table[ia, iv, :, :] - sa_r_table[ia, iv, :, :]) / pi * 180

    return stability_table, control_table, steering_sens_table, sa_diff_table


def findtrimleft(delta_range, beta_range, ay_table, ym_table, clean_table, ia, iv):
    """
    Documentation
    Subfunction of the calc function. Used to find the "trim" point for a left hand corner,
    meaning the point of highest lateral acceleration with yaw moment = 0 Nm (steady state condition)

    Input:
    ay_table:        ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table         ndarray containing the converged yaw moments of the sim [Nm]
    beta_range:      array with all beta values used in the simulation [rad]
    delta range:     array with all delta values used in the simulation [rad]
    clean_table:     ndarray with 1 oder nan to filter out points where isolines turn inwards (no function anymore)
    ia:              iterator for long. acceleration index
    iv:              iterator for long. velocity index

    Output
    intr1:           row index of point found
    intc1:           column index of point found
    """
    ay_last = 0
    ay_curr = 0

    for ic in range(len(delta_range)):
        ym_curr = ym_table[ia, iv, len(beta_range)-1, ic]

        for ir in reversed(range(len(beta_range)-1)):
            ym_last = ym_curr
            ym_curr = ym_table[ia, iv, ir, ic]

            if ym_curr * ym_last < 0:
                ay_curr = ay_table[ia, iv, ir, ic] * clean_table[ia, iv, ir, ic]

            if ay_curr > ay_last:
                ay_last = ay_curr
                intr1 = ir
                intc1 = ic
    return intr1, intc1

def findtrimright(delta_range, beta_range, ay_table, ym_table, clean_table, ia, iv):
    """
        Documentation
        Subfunction of the calc function. Used to find the "trim" point for a right hand corner,
        meaning the point of highest lateral acceleration with yaw moment = 0 Nm (steady state condition)

        Input:
        ay_table:        ndarray containing the converged lateral accelerations of the sim [m/s²]
        ym_table         ndarray containing the converged yaw moments of the sim [Nm]
        beta_range:      array with all beta values used in the simulation [rad]
        delta range:     array with all delta values used in the simulation [rad]
        clean_table:     ndarray with 1 oder nan to filter out points where isolines turn inwards (no function anymore)
        ia:              iterator for long. acceleration index
        iv:              iterator for long. velocity index

        Output
        intr2:           row index of point found
        intc2:           column index of point found
        """
    ay_last = 0
    ay_curr = 0
    for ic in range(len(delta_range)):
        ym_curr = ym_table[ia, iv, len(beta_range)-1, ic]

        for ir in range(len(beta_range)):
            ym_last = ym_curr
            ym_curr = ym_table[ia, iv, ir, ic]

            if ym_curr*ym_last < 0:
                ay_curr = ay_table[ia, iv, ir, ic] * clean_table[ia, iv, ir, ic]

            if ay_curr < ay_last:
                ay_last = ay_curr
                intr2 = ir
                intc2 = ic
    return intr2, intc2


def calc(ay_table, ym_table, ia, iv, sa_f_table, sa_r_table, beta_range, delta_range, stability_table, control_table, steering_sens_table, clean_table):
    """
        Documentation
        Calculates a set of KPI.

        Input:
        ay_table:        ndarray containing the converged lateral accelerations of the sim [m/s²]
        ym_table         ndarray containing the converged yaw moments of the sim [Nm]
        beta_range:      array with all beta values used in the simulation [rad]
        delta range:     array with all delta values used in the simulation [rad]
        clean_table:     ndarray with 1 oder nan to filter out points where isolines turn inwards (no function anymore)
        ia:              iterator for long. acceleration index
        iv:              iterator for long. velocity index
        sa_f_table:      ndarray containing the calculated slip angle at the front axle [rad]
        sa_r_table:      ndarray containing the calculated slip angle at the front axle [rad]
        stability_table:                ndarray with the stability kpi for each simulation scenario
        control_table:                  ndarray with the control kpi for each simulation scenario
        steering_sensitivity_table:     ndarray with the steering_sensitivity kpi for each simulation scenario

        Output
        kpi:             array containing the calculated kpi for the given combination of vx and ax
        """

    # KPI calculations
    sz = ay_table.shape
    kpi = np.zeros((2, 19))

    # 1: calc MZ_max
    kpi[0, 0] = np.amax(ym_table[ia, iv, :, :])
    kpi[1, 0] = np.amin(ym_table[ia, iv, :, :])

    # 2: calc ay_lim
    kpi[0, 1] = np.amax(ay_table[ia, iv, :, :])
    kpi[1, 1] = np.amin(ay_table[ia, iv, :, :])
    intr1, intc1 = np.where(ay_table[ia, iv, :, :] == kpi[0, 1])
    intr2, intc2 = np.where(ay_table[ia, iv, :, :] == kpi[1, 1])
    # in case of multiple hits choose the first one
    intr1 = intr1[0]
    intr2 = intr2[0]
    intc1 = intc1[0]
    intc2 = intc2[0]

    # 3: calc Mz @ lim
    kpi[0, 2] = ym_table[ia, iv, intr1, intc1]
    kpi[1, 2] = ym_table[ia, iv, intr2, intc2]

    # 4: SAf @ lim
    kpi[0, 3] = sa_f_table[ia, iv, intr1, intc1] / pi * 180
    kpi[1, 3] = sa_f_table[ia, iv, intr2, intc2] / pi * 180

    # 5: SAr @ lim
    kpi[0, 4] = sa_r_table[ia, iv, intr1, intc1] / pi * 180
    kpi[1, 4] = sa_r_table[ia, iv, intr2, intc2] / pi * 180

    # 6: SAdiff @ lim
    kpi[0, 5] = (sa_f_table[ia, iv, intr1, intc1] - sa_r_table[ia, iv, intr1, intc1,]) / pi * 180
    kpi[1, 5] = (sa_f_table[ia, iv, intr2, intc2] - sa_r_table[ia, iv, intr2, intc2,]) / pi * 180

    # 7: beta @ lim
    kpi[0, 6] = beta_range[intr1] / pi * 180
    kpi[1, 6] = beta_range[intr2] / pi * 180

    # 8: delta @ lim
    kpi[0, 7] = delta_range[intc1] / pi * 180
    kpi[1, 7] = delta_range[intc2] / pi * 180

    # 9: dMz / dbeta @ lim
    kpi[0, 8] = stability_table[ia, iv, intr1, intc1]
    kpi[1, 8] = stability_table[ia, iv, intr2, intc2]

    # find "trim" point. moves along constant delta lines and returns the point
    # with highest lateral acceleration and one beta step after crossing the x axis[Mz=0]

    intr1, intc1 = findtrimleft(delta_range, beta_range, ay_table, ym_table, clean_table, ia, iv)
    intr2, intc2 = findtrimright(delta_range, beta_range, ay_table, ym_table, clean_table, ia, iv)

    
    # 10: ay @ trim
    trim_1_fac_over = abs(ym_table[ia, iv, intr1 + 1, intc1]) / (abs(ym_table[ia, iv, intr1 + 1, intc1]) + abs(ym_table[ia, iv, intr1, intc1]))
    trim_1_fac_under = abs(ym_table[ia, iv, intr1, intc1]) / (abs(ym_table[ia, iv, intr1 + 1, intc1]) + abs(ym_table[ia, iv, intr1, intc1]))
    kpi[0, 9] = trim_1_fac_under * ay_table[ia, iv, intr1 + 1, intc1] + trim_1_fac_over * ay_table[ia, iv, intr1, intc1]
    
    trim_2_fac_over = abs(ym_table[ia, iv, intr2 - 1, intc2]) / (abs(ym_table[ia, iv, intr2 - 1, intc2]) + abs(ym_table[ia, iv, intr2, intc2]))
    trim_2_fac_under = abs(ym_table[ia, iv, intr2, intc2]) / (abs(ym_table[ia, iv, intr2 - 1, intc2]) + abs(ym_table[ia, iv, intr2, intc2]))
    kpi[1, 9] = trim_2_fac_under * ay_table[ia, iv, intr2 - 1, intc2] + trim_2_fac_over * ay_table[ia, iv, intr2, intc2]
    
    # 11: SAf @ trim
    kpi[0, 10] = (trim_1_fac_under * sa_f_table[ia, iv, intr1 + 1, intc1] + trim_1_fac_over * sa_f_table[ia, iv, intr1, intc1]) / pi * 180
    kpi[1, 10] = (trim_2_fac_under * sa_f_table[ia, iv, intr2 - 1, intc2] + trim_2_fac_over * sa_f_table[ia, iv, intr2, intc2]) / pi * 180
    
    # 12: SAr @ trim
    kpi[0, 11] = (trim_1_fac_under * sa_r_table[ia, iv, intr1 + 1, intc1] + trim_1_fac_over * sa_r_table[ia, iv, intr1, intc1]) / pi * 180
    kpi[1, 11] = (trim_2_fac_under * sa_r_table[ia, iv, intr2 - 1, intc2] + trim_2_fac_over * sa_r_table[ia, iv, intr2, intc2]) / pi * 180
    
    # 13: SAdiff @ trim
    kpi[0, 12] = kpi[0, 10] - kpi[0, 11]
    kpi[1, 12] = kpi[1, 10] - kpi[1, 11]
    
    # 14: beta @ trim
    kpi[0, 13] = (trim_1_fac_under * beta_range[intr1 + 1] + trim_1_fac_over * beta_range[intr1]) / pi * 180
    kpi[1, 13] = (trim_2_fac_under * beta_range[intr2 - 1] + trim_2_fac_over * beta_range[intr2]) / pi * 180
    
    # 15: delta @ trim
    kpi[0, 14] = delta_range[intc1] / pi * 180
    kpi[1, 14] = delta_range[intc2] / pi * 180
    
    # 16: dMz / dbeta @ trim
    kpi[0, 15] = trim_1_fac_under * stability_table[ia, iv, intr1 + 1, intc1]\
                         + trim_1_fac_over * stability_table[ia, iv, intr1, intc1]
    kpi[1, 15] = trim_2_fac_under * stability_table[ia, iv, intr2 - 1, intc2]\
                         + trim_2_fac_over * stability_table[ia, iv, intr2, intc2]
    
    # find point of origin[0, 0] 
    intr1 = np.where(beta_range == min(abs(beta_range)))
    intc1 = np.where(delta_range == min(abs(delta_range)))
    
    # 17: dMz / ddelta @ straight
    kpi[0, 16] = control_table[ia, iv, intr1, intc1]
    kpi[1, 16] = control_table[ia, iv, intr1, intc1]
    
    # 18: dMz / dbeta @ straight
    kpi[0, 17] = stability_table[ia, iv, intr1, intc1]
    kpi[1, 17] = stability_table[ia, iv, intr1, intc1]
    
    # 19 day / ddelta @ straight
    kpi[0, 18] = steering_sens_table[ia, iv, intr1, intc1]
    kpi[1, 18] = steering_sens_table[ia, iv, intr1, intc1]

    return kpi
