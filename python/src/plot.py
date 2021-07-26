import matplotlib.pyplot as plt
from math import nan
from math import pi
import plotly.graph_objects as go
import numpy as np
from scipy import interpolate
from scipy.interpolate import griddata
import matplotlib as mpl
import matplotlib.pyplot as plt

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""


def YMD(ay_table, ym_table, clean_table, plot_config, iv, ia):
    """
    Documentation
    Plots a yaw moment diagram with matplotlib

    Input:
    ay_table:       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table        ndarray containing the converged yaw moments of the sim [Nm]
    plot_config:    dictionary containing the configuration regarding plot layout
    ia:             iterator for long. acceleration index
    iv:             iterator for long. velocity index

    Output:
    b, c:           matplotlib axes objects
    """

    plt.figure(figsize=plot_config["figsz"])

    b = [None] * ay_table.shape[2]
    for r in range(ay_table.shape[2]):
        if r == 0:
            b[r] = plt.plot(ay_table[ia, iv, r, :] * clean_table[ia, iv, r, :], ym_table[ia, iv, r, :] *
                            clean_table[ia, iv, r, :], color='b', linewidth=0.4, label='constant beta')
        b[r] = plt.plot(ay_table[ia, iv, r, :]* clean_table[ia, iv, r, :], ym_table[ia, iv, r, :] *
                        clean_table[ia, iv, r, :], linewidth=0.4, color='b')

    d = [None] * ay_table.shape[3]
    for c in range(ay_table.shape[3]):
        if c == 0:
            d[c] = plt.plot(ay_table[ia, iv, :, c] * clean_table[ia, iv, :, c], ym_table[ia, iv, :, c] *
                            clean_table[ia, iv, :, c], color='r', linewidth=0.4, label='constant delta')
        d[c] = plt.plot(ay_table[ia, iv, :, c] * clean_table[ia, iv, :, c], ym_table[ia, iv, :, c] *
                        clean_table[ia, iv, :, c], linewidth=0.4, color='r')

    plt.title("Yaw Moment Diagram")
    plt.xlabel("lateral acceleration [m/s²]")
    plt.ylabel("Yaw Moment [Nm]")
    plt.minorticks_on()
    plt.grid(which='minor', linewidth=0.2)
    plt.grid(which='major', linewidth=0.4)
    plt.legend()
    plt.xlim([-plot_config["x_lim"], plot_config["x_lim"]])
    plt.ylim([-plot_config["y_lim"], plot_config["y_lim"]])

    return b, c


def YMDplotly(ay_table, ym_table, clean_table, ax_range, v_range, beta_range, delta_range, plot_config, ia):
    """
    Documentation
    Plots a yaw moment diagram with over a given velocity range with plotly

    Input:
    ay_table:       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table        ndarray containing the converged yaw moments of the sim [Nm]
    clean_table     ndarray with 1 oder nan to filter out points where isolines turn inwards (no function anymore)
    ax_range:       array with all ax values used in the simulation [rad]
    vx_range:       array with all vx values used in the simulation [rad]
    beta_range:     array with all beta values used in the simulation [rad]
    delta range:    array with all delta values used in the simulation [rad]
    plot_config:    dictionary containing the configuration regarding plot layout
    ia:             iterator for long. acceleration index

    Output:
    fig:            plotly figure object
    """
    sz = ay_table.shape

    title = f'Yaw Moment Diagram @ ax={ax_range[ia]}m/s²'
    labels_delta = [None] * sz[3]
    labels_beta = [None] * sz[2]

    for i in range(len(delta_range)):
        labels_delta[i] = f'\u03B4={(delta_range[i] / pi * 180).round(3)}\xb0'
    for i in range(len(beta_range)):
        labels_beta[i] = f'\u03B2={(beta_range[i] / pi * 180).round(3)}\xb0'
    colors = ['rgb(0,0,160)', 'rgb(160,0,0)']

    line_size = 2

    x_data = ay_table[ia, :, :, :]

    y_data = ym_table[ia, :, :, :]

    clean_table = clean_table[ia, :, :, :]

    fig = go.Figure()

    for iv in range(len(v_range)):
        for i in range(sz[2]):
            fig.add_trace(go.Scatter(x=x_data[iv, i] * clean_table[iv, i], y=y_data[iv, i] * clean_table[iv, i],
                                     mode='lines', name=labels_beta[i], visible=False,
                                     line=dict(color=colors[0], width=line_size),
                                     connectgaps=False,
                                     ))

        for i in range(sz[3]):
            fig.add_trace(go.Scatter(x=x_data[iv, :, i] * clean_table[iv, :, i], y=y_data[iv, :, i] * clean_table[iv, :, i],
                                     mode='lines', name=labels_delta[i], visible=False,
                                     line=dict(color=colors[1], width=line_size),
                                     connectgaps=False,
                                     ))

    # pack visibilty range list
    vis = np.zeros((len(v_range), 2), dtype='int16')
    inc = len(beta_range) + len(delta_range)
    val = inc
    vis[0, 1] = val
    for i in range(1, len(v_range)):
        vis[i, 0] = val
        val = val + inc
        vis[i, 1] = val

    # Make 1st diagram visible
    for i in range(vis[0, 0], vis[0, 1]):
        fig.data[i].visible = True

    # Create and add slider
    steps = []
    for i in range(len(v_range)):
        step = dict(
            method="update",
            args=[{"visible": [False] * len(fig.data)}]  # layout attribute
        )
        for itr in range(vis[i, 0], vis[i, 1]):
            step["args"][0]["visible"][itr] = True  # Toggle i'th trace to "visible"
            step["label"] = f'{v_range[i]} m/s'
        steps.append(step)

    sliders = [dict(
        currentvalue={"prefix": "Displaying velocity: ", "font": {"size": 15}},
        pad={"t": 80, "r": 250, "l": 250},
        name="velocity",
        steps=steps,

    )]

    fig.update_layout(
        sliders=sliders
    )

    fig.update_layout(
        xaxis=dict(
            title_text='lateral acceleration [m/s²]',
            range=[-plot_config["x_lim"], plot_config["x_lim"]],
            showline=True,
            showgrid=True,
            showticklabels=True,
            linecolor='rgb(204, 204, 204)',
            linewidth=2,
            ticks='outside',
            tickfont=dict(
                family='Arial',
                size=15,
                color='rgb(82, 82, 82)',
            ),
        ),
        yaxis=dict(
            title_text='yaw moment [Nm]',
            range=[-plot_config["y_lim"], plot_config["y_lim"]],
            showline=True,
            showgrid=True,
            showticklabels=True,
            linecolor='rgb(204, 204, 204)',
            linewidth=2,
            ticks='outside',
            tickformat='g',
            tickfont=dict(
                family='Arial',
                size=15,
                color='rgb(82, 82, 82)',
            ),
        ),
        hovermode="x",
        hoverdistance=2,
        autosize=True,
        showlegend=False,
    )

    annotations = []

    # Title
    annotations.append(dict(xref='paper', yref='paper', x=0.0, y=1.05,
                            xanchor='left', yanchor='bottom',
                            text=title,
                            font=dict(family='Arial',
                                      size=20,
                                      color='rgb(37,37,37)'),
                            showarrow=False))

    fig.update_layout(annotations=annotations)

    # fig.show()

    return fig


def contourplotly(ay_table, ym_table, z_table, clean_table, ax_range, v_range, beta_range, delta_range, plot_config,
                  ia):

    sz = ay_table.shape
    ay_table = ay_table[0, 0, 15, :]
    ym_table = ym_table[0, 0, 15, :]
    z_table = z_table[0, 0, 15, :]

    ay_table_flat = ay_table.flatten()
    ym_table_flat = ym_table.flatten()
    z_table_flat = z_table.flatten()

    # z_table = np.nan_to_num(z_table)
    f = interpolate.interp2d(ay_table_flat, ym_table_flat, z_table_flat)
    x2 = np.linspace(-40, 40, 30)
    y2 = np.linspace(-40000, 40000, 30)
    z2 = f(x2, y2)
    """""
    def pointValue(x, y, power, smoothing, x2, y2, values):
        nominator = 0
        denominator = 0
        for i in range(0, len(values)):
            dist = sqrt((x - x2[i]) * (x - x2[i]) + (y - y2[i]) * (y - y2[i]) + smoothing * smoothing)
            # If the point is really close to one of the data points, return the data point value to avoid singularities
            if (dist < 0.0000000001):
                return values[i]
            nominator = nominator + (values[i] / pow(dist, power))
            denominator = denominator + (1 / pow(dist, power))
        # Return NODATA if the denominator is zero
        if denominator > 0:
            value = nominator / denominator
        else:
            value = -9999
        return value

    def invDist(x2, y2, values, xsize, ysize, power, smoothing):
        valuesGrid = np.zeros((ysize, xsize))
        for x in range(xsize):
            for y in range(ysize):
                valuesGrid[y][x] = pointValue(x, y, power, smoothing, x2, y2, values)
        return valuesGrid


    power = 1
    smoothing = 20

    # Creating the interpolation function and populating the output matrix value
    z4 = invDist(x2, y2, z_table, 500, 500, power, smoothing)
    """

    z3 = griddata((ay_table_flat, ym_table_flat), z_table_flat, (x2, y2), method='cubic')

    # z_table = np.ones((sz[2], sz[3]))
    fig = go.Figure(data=
    go.Contour(
        z=z2,
        x=x2,  # horizontal axis
        y=y2  # vertical axis
    ))
    fig.show()
    return fig


def contour(ay_table, ym_table, z_table, clean_table, plot_config, ia, iv):
    """
    Documentation
    Plots a countour plot with a a colorbar where white is always at 0

    Input:
    ay_table:       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table        ndarray containing the converged yaw moments of the sim [Nm]
    z_table:        ndarray containing any kpi in the same ashape as ay_table and ym_table
    clean_table:    ndarray with 1 oder nan to filter out points where isolines turn inwards (no function anymore)
    plot_config:    dictionary containing the configuration regarding plot layout
    ia:             iterator for long. acceleration index
    iv:             iterator for long. velocity index

    Output:
    fig:            matplotlib figure
    cb:             matplotlib colorbar
    """

    plt.figure(figsize=plot_config["figsz"])

    #Normalize colorbar (white=0)
    class MidpointNormalize(mpl.colors.Normalize):
        def __init__(self, vmin, vmax, midpoint=0, clip=False):
            self.midpoint = midpoint
            mpl.colors.Normalize.__init__(self, vmin, vmax, clip)

        def __call__(self, value, clip=None):
            normalized_min = max(0, 1 / 2 * (1 - abs((self.midpoint - self.vmin) / (self.midpoint - self.vmax))))
            normalized_max = min(1, 1 / 2 * (1 + abs((self.vmax - self.midpoint) / (self.midpoint - self.vmin))))
            normalized_mid = 0.5
            x, y = [self.vmin, self.midpoint, self.vmax], [normalized_min, normalized_mid, normalized_max]
            return np.ma.masked_array(np.interp(value, x, y))

    z_table_num = np.nan_to_num(z_table[ia, iv, :, :])

    vmin = z_table_num.min()
    vmax = z_table_num.max()

    norm = MidpointNormalize(vmin=vmin, vmax=vmax, midpoint=0)

    # use clean table to make plots better readable
    ay_table = ay_table[ia, iv, :, :] * clean_table[ia, iv, :, :]
    ym_table = ym_table[ia, iv, :, :] * clean_table[ia, iv, :, :]
    z_table = z_table[ia, iv, :, :] * clean_table[ia, iv, :, :]

    # plot
    plt.figure(1)
    fig = plt.contourf(ay_table, ym_table, z_table, levels=20, cmap='RdBu', norm=norm)

    # set labels and descriptions
    plt.xlabel('lateral acceleration [m/s^{2}]')
    plt.ylabel('yawm oment [Nm]')
    plt.grid()
    plt.xlim([-plot_config["x_lim"], plot_config["x_lim"]])
    plt.ylim([-plot_config["y_lim"], plot_config["y_lim"]])
    cb = plt.colorbar(fig)


    return fig, cb


def contour2(ay_table, ym_table, z_table, clean_table, plot_config, ia, iv):
    """
    Documentation
    Plots a countour plot with a a colorbar with default colors

    Input:
    ay_table:       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ym_table        ndarray containing the converged yaw moments of the sim [Nm]
    z_table:        ndarray containing any kpi in the same ashape as ay_table and ym_table
    clean_table:    ndarray with 1 oder nan to filter out points where isolines turn inwards (no function anymore)
    plot_config:    dictionary containing the configuration regarding plot layout
    ia:             iterator for long. acceleration index
    iv:             iterator for long. velocity index

    Output:
    fig:            matplotlib figure
    cb:             matplotlib colorbar
    """

    plt.figure(figsize=plot_config["figsz"])

    ay_table = ay_table[ia, iv, :, :] * clean_table[ia, iv, :, :]
    ym_table = ym_table[ia, iv, :, :] * clean_table[ia, iv, :, :]
    z_table = z_table[ia, iv, :, :] * clean_table[ia, iv, :, :]

    plt.figure(1)
    fig = plt.contourf(ay_table, ym_table, z_table, levels=20)

    plt.xlabel('lateral acceleration [m/s^{2}]')
    plt.ylabel('yaw moment [Nm]')
    plt.grid()
    plt.xlim([-plot_config["x_lim"], plot_config["x_lim"]])
    plt.ylim([-plot_config["y_lim"], plot_config["y_lim"]])
    cb = plt.colorbar(fig)

    return fig, cb


def clean_table(ay_table, ia, iv):
    """
    Documentation
    Calculates a boolian array to clean yaw moment diagram isolines when they would not be a function anymore

    Input:
    ay_table:       ndarray containing the converged lateral accelerations of the sim [m/s²]
    ia:             iterator for long. acceleration index
    iv:             iterator for long. velocity index

    Output:
    clean_table:    ndarray with 1 and NaN values
    """

    sz = ay_table.shape
    clean_table = np.zeros((sz[2], sz[3]))
    # clean table for constant delta lines
    # will be multiplied with data table for cutting off plot lines
    # to make the horizontal edges of the diagram easier to asses

    for ic in range(sz[3]):
        lock = 0
        for ir in range(round(sz[2] / 2), -1, -1):
            ay_curr = ay_table[ia, iv, ir, ic]

            if ir == 0:
                ay_next = ay_curr
            else:
                ay_next = ay_table[ia, iv, ir - 1, ic]

            if ay_next < ay_curr or lock == 1:
                clean_table[ir, ic] = nan
                lock = 1
            else:
                clean_table[ir, ic] = 1

        lock = 0
        for ir in range(round(sz[2] / 2), sz[2]):
            ay_curr = ay_table[ia, iv, ir, ic]

            if ir == 1:
                ay_last = ay_curr
            else:
                ay_last = ay_table[ia, iv, ir - 1, ic]

            if ay_last < ay_curr or lock == 1:
                clean_table[ir, ic] = nan
                lock = 1
            else:
                clean_table[ir, ic] = 1

    return clean_table
