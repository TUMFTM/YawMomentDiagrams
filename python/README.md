# Generation of Yaw Moment Diagrams in Python

## List of software components
* `params`: Contains the parameter files to configure the simulation
* `src`: Contains the subscripts of main_calcYMD.m and additional helper scripts
* `vehicledynamics_wheel`: Contains the Python wheel which allows to include the vehicle dynamics model in Python

**Note: The provided python wheel can only be used with Python running on Linux!**

## How to generate Yaw Moment Diagrams
1. Create a virtual environment
2. Install the required packages via ``pip install -r requirements.txt``
3. Install the TUM_vehicle_dynamics.whl file located in ``./vehicledynamics_wheel`` via ``pip install tum_veh... .whl``
4. Modify the parameter files located in the ``./params`` folder.
5. Run ``main_calcYMD.m``. Output files are created in the ``output`` folder (folder is created if it does not exist).

## Analyzing the results
A general description is provided in the top-level README.md of this repository at "Analysis of generated Yaw Moment Diagrams".

Additional remarks:
* The ``params`` folder contains a copy of the param files used during the simulation.

Additionally there will be a ``output.mat`` file to do a KPI analysis.
The ``plot_KPI.py`` script in the src folder can be used to do this. In there you specify the directories of the different simulation runs you want to compare.
For further information you can consult the Quick Guide in the header of the ``plot_KPI.py`` script.
