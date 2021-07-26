# Generation of Yaw Moment Diagrams in Matlab/Simulink

## List of software components
* `models`: Contains the model which will be used by main_calcYMD.m and which references the submodels nonlineardtm and nlstm
* `src`: Contains functions called by main_calcYMD.m and additional helper scripts

## How to generate Yaw Moment Diagrams
1. Clone our [vehicle dynamics simulation repository](https://github.com/TUMFTM/sim_vehicle_dynamics).
2. Once cloned, open ``sim_vehicle_dynamics.prj`` there.
3. Run ``configureSimParams('<VehicleIdentifier>')`` in Matlab command line to load the desired vehicle parameters, e.g. ``configureSimParams('pa')``.
4. Open ``main_calcYMD.m`` and set parameters in the script's header.
5. Run ``main_calcYMD.m``. Output files are created in the ``output`` folder (folder is created if it does not exist).

Alternatively, the YMD generation can be run via ``main_calcYMD_parallel.m``. However, this is only faster when running a more extensive analysis since setting up the parallel workers is also time consuming.
If you want to change any vehicle parameters, open the vehicle parameter data dictionary of the ``sim_vehicle_dynamics`` repository where you can find and alter any vehicle parameter. Further details regarding the vehicle parameters can be found in the other repository's docs.

## Analyzing the results
A general description is provided in the top-level README.md of this repository at "Analysis of generated Yaw Moment Diagrams".

Additional remarks:
* For every longitudinal acceleration there will be a 3D Velocity Diagram (outer edges of the maneuvering space of the YMD of every velocity step merged into one diagram).
* The ``DataDicitionary.mat`` file is a copy of the data dictionary with the parameter set used during the simulation.

Additionally there will be a ``output.mat`` file to do a KPI analysis.
The ``plot_KPI.m`` script in the src folder can be used to do this. In there you specify the directories of the different simulation runs you want to compare.
For further information you can consult the Quick Guide in the header of the ``plot_KPI.m`` script.
