import os
import sys
from scipy.io import loadmat
import numpy as np
import plotly.graph_objects as go

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""

def load(path):
    """
    Documentation
    Loads the output.mat files of the specified dictionaries

    Input:
    veh:                    TUM_vehicle_dynamics package

    Output:
    kpi_dir:                ndarray of all kpi loaded
    path_split:             all names of the directories loaded
    data:                   copy of the last output.mat file loaded
    """

    # Split directory path by '\'
    path_split = []
    for i in range(len(path)):
        path_split.append(os.path.split(path[i]))

    # load data from first directory
    # check if path is valid
    if os.path.isdir(path[0]):
        # check if output.mat file is in path
        try:
            data = loadmat(os.path.join(path[0], "output.mat"))
        except:
            sys.exit(f'Specified path {path[0]} does not contain a file "output.mat"')
        # import kpi values
        sz = data["kpi"].shape
        sz2 = len(path)
        kpi_dir = np.zeros((sz2, sz[0], sz[1], sz[2], sz[3]))
        kpi_dir[0, :, :, :, :] = data["kpi"]
    else:
        sys.exit(f'Specified path {path[0]} does not exist')

    # load data from following directories
    for i in range(1, sz2):
        # check if path is valid
        if os.path.isdir(path[i]):
            del data
            # check if output.mat file is in path
            try:
                data = loadmat(os.path.join(path[i], "output.mat"))
            except:
                sys.exit(f'Specified path {path[i]} does not contain a file "output.mat"')
            # check dimensions
            sz1 = kpi_dir.shape
            sz2 = data["kpi"].shape
            if sz1[1] == sz2[0] and sz1[2] == sz2[1] and sz1[4] == sz2[3]:
                # import kpi values
                kpi_dir[i, :, :, :, :] = data["kpi"]
            else:
                sys.exit('"output.mat" file dimensions do not agree. Make sure you used the same album config files. (See "main_calcYMD.py")')
        else:
            sys.exit(f'Specified path {path[i]} does not exist')

    return kpi_dir, path_split, data

def bargraph_plotly(kpi, direction, path_split, data):
    """
    Documentation
    Shows the loaded KPI in a plotly bargraph

    Input:
    kpi:                    ndarray of all kpi loaded
    direction:              indicator for left or right hand turns (LH=0, RH=1)
    path_split:             all names of the directories loaded
    data:                   copy of the last output.mat file loaded

    Output:
    fig:                    Plotly figure object
    """
    # initialization
    kpi_count = 19
    kpi_descr = [None]*kpi_count
    kpi_unit = [None]*kpi_count
    d = direction
    sz = kpi.shape
    x_data = []
    v_range = data["v_range"]
    ax_range = data["ax_range"]
    y_data = []

    # General KPI
    kpi_descr[0] = 'Mz max'
    kpi_unit[0] = 'Maximum Yaw Moment [Nm]'

    # for the following KPI 'lim' corresponds to the point of maximum lateral acceleration
    kpi_descr[1] = 'ay@lim'
    kpi_unit[1] = 'maximum lateral acceleration [m/s²]'
    kpi_descr[2] = 'Mz@lim'
    kpi_unit[2] = 'yaw moment @ lim [Nm]'
    kpi_descr[3] = 'SAf@lim'
    kpi_unit[3] = 'slip angle front @ lim [\xb0]'
    kpi_descr[4] = 'SAr@lim'
    kpi_unit[4] = 'slip angle rear @ lim [\xb0]'
    kpi_descr[5] = 'SAdiff@lim'
    kpi_unit[5] = 'slip angle difference @ lim [\xb0]'
    kpi_descr[6] = '\u03B2@lim'
    kpi_unit[6] = 'beta @ lim [\xb0]'
    kpi_descr[7] = '\u03B4@lim'
    kpi_unit[7] = 'delta @ lim [\xb0]'
    kpi_descr[8] = 'dMz/d\u03B2@lim'
    kpi_unit[8] = 'stability @ lim [Nm/\xb0]'

    # for the following KPI 'trim' corresponds to the point of maximum lateral
    # acceleration achievable in steady state[trimmed condition] --> yawmoment = 0 Nm
    kpi_descr[9] = 'ay@trim'
    kpi_unit[9] = 'maximum lateral acceleration in trimmed condition [m/s²]'
    kpi_descr[10] = 'SAf@trim'
    kpi_unit[10] = 'slip angle front @ trim [\xb0]'
    kpi_descr[11] = 'SAr@trim'
    kpi_unit[11] = 'slip angle rear @ trim [\xb0]'
    kpi_descr[12] = 'SAdiff@trim'
    kpi_unit[12] = 'slip angle difference @ trim [\xb0]'
    kpi_descr[13] = '\u03B2@trim'
    kpi_unit[13] = 'beta @ trim [\xb0]'
    kpi_descr[14] = '\u03B4@trim'
    kpi_unit[14] = 'delta @ trim [\xb0]'
    kpi_descr[15] = 'dMz/d\u03B2@trim'
    kpi_unit[15] = 'stability @ trim [Nm/\xb0]'

    # for the following KPI 'straight' corresponds to delta=0 and beta=0, e.g.
    # driving in a straight line
    kpi_descr[16] = 'dMz/d\u03B4@straight'
    kpi_unit[16] = 'controllability @ straight [Nm/\xb0]'
    kpi_descr[17] = 'dMz/d\u03B2@straight'
    kpi_unit[17] = 'stability @ straight [Nm/\xb0]'
    kpi_descr[18] = 'day/d\u03B4@straight'
    kpi_unit[18] = 'steering sensitivity @ straight [ay/\xb0]'

    fig = go.Figure()

    # assemble vector with x axis description
    for v in range(sz[2]):
        for a in range(sz[1]):
            x_data.append(f'v={v_range[0, v]}m/s, ax={ax_range[0, a]}m/s²')

    #define bar traces
    for k in range(sz[4]):
        for p in range(sz[0]):
            for v in range(sz[2]):
                for a in range(sz[1]):
                    y_data.append(kpi[p, a, v, d, k])
            fig.add_trace(go.Bar(name=path_split[p][1], x=x_data, y=y_data, visible=False))
            y_data = []
    
    # pack visibilty range list for slider control
    inc = sz[0]
    length = inc * sz [4]
    vis = np.zeros((length, 2), dtype='int16')
    val = inc
    vis[0, 1] = val
    for i in range(1, sz[4]):
        vis[i, 0] = val
        val = val + inc
        vis[i, 1] = val


    # Make 1st diagram visible
    for i in range(vis[0, 0], vis[0, 1]):
        fig.data[i].visible = True


    # Create and add slider
    steps = []
    for i in range(sz[4]):
        step = dict(
            method="update",
            args=[{"visible": [False] * len(fig.data)},
                  {"yaxis.title": kpi_unit[i], "yaxis.title.font.size": 15}]
            )
        for itr in range(vis[i, 0], vis[i, 1]):
            step["args"][0]["visible"][itr] = True  # Toggle i'th trace to "visible"
            step["label"] = kpi_descr[i]
        steps.append(step)

    sliders = [dict(
        currentvalue={"prefix": "Displaying KPI: ", "font": {"size": 15}},
        pad={"t": 80},
        steps=steps,
        font={"size": 10}

    )]

    fig.update_layout(
        sliders=sliders
    )

    # Title
    annotations = []
    annotations.append(dict(xref='paper', yref='paper', x=0.0, y=1.05,
                            xanchor='left', yanchor='bottom',
                            text="KPI comparison",
                            font=dict(family='Arial',
                                      size=20,
                                      color='rgb(37,37,37)'),
                            showarrow=False))

    fig.update_layout(annotations=annotations)

    return fig
