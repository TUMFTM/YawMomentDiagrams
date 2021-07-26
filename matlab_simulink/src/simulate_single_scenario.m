%%This script uses the ymd.slx to simulate a transient situation in a
%static manner. You can set target values for delta and beta. The beta is
%controlled during this simulation using a external yaw moment controller.

%%
%Quick Guide:
%Open Project sim_vehicle_dynamics.prj
%Open ymd.slx and add to matlab path
%run script
%configureSimParams('db')

%%
%target values
sim_model=1; %1=nldtm; 2=nlstm_V2; 3=nldtm_V2
banking=0; %Banking angle in deg, neg. values for left hand corner banking
brake_balance=0.8; %ratio front
tv_factor=0; %Torque Vectoring
v=60; %mps
ax=0; %mps2
delta=0.2; %rad
beta=0; %rad

inti_b=1;
inti_d=1;
inti_v=1;
inti_a=1;

set_parameter %adjust parameter in data dictionary

sim('ymd')
  
       