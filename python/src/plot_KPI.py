import src

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021

Documentation:
Main script to plot and compare KPI

Requirements:
1. at least one path containing a output.mat must be given 
2. if given multiple path, the different albums must be created with the same album config file (see main_calcYMD)
"""

path = []
# USER INPUT
# ______________________________________________________________________________________________________________________
direction = 0  # direction of turn 0==LH turn 1== RH turn

path.append('/home/frederik/Dokumente/Plots')
path.append('/home/frederik/Dokumente/Plots2')
path.append('/home/frederik/Dokumente/Plots3')

# ______________________________________________________________________________________________________________________

kpi, path_split, data = src.kpi_handling.load(path)
fig = src.kpi_handling.bargraph_plotly(kpi, direction, path_split, data)
fig.show()
