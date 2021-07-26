%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme file or the Quick Guide at the start
%of the 'create_YMD album' script

%%

%open Data Dictionary linked to Sim
DD = Simulink.data.dictionary.open('vehicleparameters_vehdynsim.sldd');
DataSect = getSection(DD,'Design Data');

%add new Entries for delta and beta or change them to target values

if exist(DataSect,'YMD_set_beta')==false
    addEntry(DataSect,'YMD_set_beta',beta);
else
    entryObj=getEntry(DataSect, 'YMD_set_beta');
    setValue(entryObj, beta);
end

if inti_b==1
    if exist(DataSect,'YMD_set_delta')==false
        addEntry(DataSect,'YMD_set_delta',delta);
    else
        setValue(getEntry(DataSect, 'YMD_set_delta'), delta);
    end
end

%set initial velocity
if inti_d==1
    if exist(DataSect,'INIT__initialvalues__Vehicle_InitialVelocity_mps__1')==false
        addEntry(DataSect,'INIT__initialvalues__Vehicle_InitialVelocity_mps__1',v);
    else
        setValue(getEntry(DataSect, 'INIT__initialvalues__Vehicle_InitialVelocity_mps__1'), v);
    end

    %calculate start wheel rotaion speeds by initial velocity
    r_front=getValue(getEntry(DataSect,'tireFL__MFSIMPLE__r_tire_m'));
    r_rear=getValue(getEntry(DataSect,'tireRL__MFSIMPLE__r_tire_m'));

    u_front=r_front*2*pi();
    u_rear=r_rear*2*pi();

    omega_front=(v/u_front)*2*pi();
    omega_rear=(v/u_rear)*2*pi();

    %set inital wheel rotation
    setValue(getEntry(DataSect, 'INIT__initialvalues__omega0_Wheels_radps__1'), omega_front);
    setValue(getEntry(DataSect, 'INIT__initialvalues__omega0_Wheels_radps__2'), omega_front);
    setValue(getEntry(DataSect, 'INIT__initialvalues__omega0_Wheels_radps__3'), omega_rear);
    setValue(getEntry(DataSect, 'INIT__initialvalues__omega0_Wheels_radps__4'), omega_rear);
end

%calulate external force for required ax
if inti_v==1
    mass_veh=getValue(getEntry(DataSect,'VEH__VehicleData__m_Vehicle_kg'));
    fx_ext=-mass_veh*ax; %neg because to archieve positive ax vehicle need to be constrained with a negative force in order for the simulation to be steady state

    % write calculated Fx to data dictionary
    if exist(DataSect,'YMD_set_fx')==false
        addEntry(DataSect,'YMD_set_fx',fx_ext);
    else
        setValue(getEntry(DataSect, 'YMD_set_fx'), fx_ext);
    end
end


if inti_a==1
    %convert Banking angle to rad
    banking_angle=banking/180*pi();

    % write calculated banking_angle to data dictionary
    if exist(DataSect,'YMD_set_banking')==false
         addEntry(DataSect,'YMD_set_banking',banking_angle);
    else
         setValue(getEntry(DataSect, 'YMD_set_banking'), banking_angle);
    end

    % write Torque Vectoring setting to data dictionary
    if exist(DataSect,'YMD_set_tv')==false
         addEntry(DataSect,'YMD_set_tv',tv_factor);
    else
         setValue(getEntry(DataSect, 'YMD_set_tv'), tv_factor);
    end

    % write Brake Balance setting to data dictionary
    if exist(DataSect,'YMD_set_bb')==false
         addEntry(DataSect,'YMD_set_bb',brake_balance);
    else
         setValue(getEntry(DataSect, 'YMD_set_bb'), brake_balance);
    end

    % write sim model selection to data dictionary
    if exist(DataSect,'YMD_sim_model')==false
         addEntry(DataSect,'YMD_sim_model',sim_model);
    else
         setValue(getEntry(DataSect, 'YMD_sim_model'), sim_model);
    end
end