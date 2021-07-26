%Script to evaluate the peak values of delta and beta

clear
configureSimParams('db')
warning('off','all') %Disable Simulink Warnings
set_param('ymd','SimulationMode','accelerator') %Simulate in "Accelerator Mode"
set_param('ymd','FastRestart','on')  %Simulate with Fast Restart

%target values
v=20; %mps
ax=-5; %mps2
banking=0; %Banking angle in deg, neg. values for left hand corner banking
brake_balance=0.8; %ratio front
tv_factor=50; %Torque Vectoring

%starting points
delta=0; %rad
beta=0; %rad
ay_last=0;
ay_curr=0;

%loop for delta_max
while ay_last<=ay_curr
    delta=delta+0.01;
    set_parameter;
    simOut=sim('ymd');
    SimRealState=get(simOut, 'SimRealState');
    ay_last=ay_curr;
    ay_data=getdatasamples(SimRealState.ay_mps2,[4500:5000]);
    ay_curr=mean(ay_data);
end
delta_max=delta-0.01
delta=0;
ay_last=0;
ay_curr=0;
%loop for beta_max
while ay_last<=ay_curr
    beta=beta-0.005;
    set_parameter;
    simOut=sim('ymd');
    SimRealState=get(simOut, 'SimRealState');
    ay_last=ay_curr;
    ay_data=getdatasamples(SimRealState.ay_mps2,[4500:5000]);
    ay_curr=mean(ay_data);
end
beta_max=beta+0.005

set_param('ymd','FastRestart','off')
set_param('ymd','SimulationMode','normal')