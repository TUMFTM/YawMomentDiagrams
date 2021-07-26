% Quick Guide:
% 1. Open sim_vehicle_dynamics.prj
% 2. Set configuration values in this script
% 3. Run

clearvars

%% USER INPUT

    %configuration of simulation
    sim_model=1; %1=nldtm; 2=nlstm_V2; 3=nldtm_V2
    banking=0; %Banking angle in deg (negative values relate to left hand corner banking)
    brake_balance=0.8; %ratio front
    tv_factor=0; %Torque Vectoring (Gain to calculate a Mz through different wheel torque distribution
    %--> ay * tv_factor= target Mz

    %definition of the desired speed values for the sim
    v_min=20; %mps
    v_max=80; %mps
    v_steps=4; %integer

    %definition of the desired long. acceleration values for the sim
    ax_min=-5; %mps2
    ax_max=5; %mps2
    ax_steps=3; %integer

    %definition of the desired delta range used in the simulation of each plot
    %(Note: will always be mirrored into negative)
    delta_max=0.15; %rad (Suggestion:  Based on the tire model used, this value should be sufficiently
    %high to cover all peaks in the Fy-slipangle diagram. Note: A too-high value will not invalidate
    %the results but will lead to a more coarse diagramm. A too-low value will mean that the diagrams
    %will not be complete because not all points in the maneurvering space can be reached.)
    delta_steps=7; %integer

    %definition of the desired beta range used in the simulation of each plot
    %(Note: will always be mirrored into negative)
    beta_max=0.105; %rad (Same tip as for the delta_max value)
    beta_steps=15; %integer

    %axis limits for plot generation
    x_lim=40; %[m/s^2]
    y_lim=40000; %[Nm]

%End of Parameter Configuration and start of non editable code
%% Initialization

 	% manage paths
    % directory in which the plots will be saved
    path = split(which("main_calcYMD.m"), "main_calcYMD.m");
    foldername = horzcat(char(datetime('now', 'Format', 'yyyy-MM-dd')), '_', char(datetime('now', 'Format', 'HH-mm-ss')));
    path2outputs = fullfile(path{1}, 'output', foldername);
    
    % check if output folder exists
    if not(isfolder(path2outputs))
        mkdir(path2outputs)
    end
    
    % add paths which contain src files and slx model
    addpath(fullfile(path{1}, 'models'))
    addpath(fullfile(path{1}, 'src'))

    % get filepath of ymd simulink model
    model = "ymd";
    filepath2ymd_model = which(model);

    % open ymd simulink model
    disp('Open Model')
    open_system(filepath2ymd_model)

    %disable warnings
    warning('off','all') %Disable Simulink Warnings

%% main YMD calculation

    disp('----------------------------------------------------')
    disp('Start YMD calculation!')

    %create range
    delta_steps=delta_steps*2+1;
    beta_steps=beta_steps*2+1;
    delta_min=-delta_max; %mirror delta range into negative
    beta_min=-beta_max; %mirror beta range into negative
    v_range=linspace(v_min, v_max, v_steps); %linear interpolation of velocity range
    ax_range=linspace(ax_min, ax_max, ax_steps); %linear interpolation of long. accel. range
    delta_range=linspace(delta_min, delta_max, delta_steps); %linear interpolation of delta range
    beta_range=linspace(beta_min, beta_max, beta_steps); %linear interpolation of beta range
    
    %Initialize waitbar
    f = waitbar(0,'Initializing...');
    overall_length=max(size(ax_range))*max(size(v_range))*max(size(delta_range))*max(size(beta_range));
    z = 0;

    %configure sim
    set_param('ymd','SimulationMode','accelerator') %Simulate in "Accelerator Mode"
    set_param('ymd','FastRestart','off')
    % Find all ports with data logging enabled
    pH = find_system(model, 'LookUnderMasks','all','FindAll', 'on', 'DataLogging', 'on');
    % Disable all logging
    for x=1:length(pH)
       set_param(pH(x), 'DataLogging', 'off');
    end

    %initialize Variables
    inti_v=1; %internal Variable for velocity loop
    inti_a=1; %internal Variable for acceleration loop
    inti_b=1; %internal Variable for beta loop
    inti_d=1; %internal Variable for delta loop

    %initialize 4D Matrices
    ay_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    dphi_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    ay_std_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    ym_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_fl_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_fr_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_rl_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_rr_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_f_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_r_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sa_delta_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    stability_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    sideslip_damping_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    control_moment_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    control_force_table=zeros(beta_steps, delta_steps, v_steps, ax_steps);
    kpi=zeros(2, 19, v_steps, ax_steps);

    %create one plot for each velocity in v_range and ax in ax_range and save to directory

    %loop for velocity
    for inti_v=1:max(size(v_range))
        v=v_range(inti_v);
        %loop for long. accel.
        for inti_a=1:max(size(ax_range))
            ax=ax_range(inti_a);
            set_param('ymd','FastRestart','on')
            %loop for delta
            for inti_d=1:max(size(delta_range))

                %flip plus/minus sign in ay_table for single track model
                if sim_model==2 ||sim_model==3
                    delta=delta_range(inti_d);
                else
                    delta=delta_range(inti_d);
                end

                for inti_b=1:max(size(beta_range))

                    %flip plus/minus sign in ay_table for single track model
                    if sim_model==2 || sim_model==3
                        beta=-beta_range(inti_b);
                    else
                        beta=beta_range(inti_b);
                    end

                    %run simulation and retrieve outputs needed
                    set_parameter;
                    simOut=sim('ymd');
                    YMD_extYawMoment=get(simOut, 'YMD_extYawMoment');
                    ay_mps2=get(simOut, 'ay_mps2');
                    dphi_radps=get(simOut, 'dphi_radps');
                    alpha_Tires_rad=get(simOut, 'alpha_rad');

                    %Extract mean value of ay, yawmoment and slip angles of the last second
                    %of the sim
                    ym_data=getdatasamples(YMD_extYawMoment,4500:5000);
                    ym_mean=mean(ym_data);
                    dphi_data=getdatasamples(dphi_radps,4500:5000);
                    dphi_mean=mean(dphi_data);
                    ay_data=getdatasamples(ay_mps2,4500:5000);
                    ay_mean=mean(ay_data);
                    ay_std=std(ay_data);
                    sa_data=getdatasamples(alpha_Tires_rad,4500:5000);
                    sa_fl_mean=mean(sa_data(:,1,:));
                    sa_fr_mean=mean(sa_data(:,2,:));
                    sa_rl_mean=mean(sa_data(:,3,:));
                    sa_rr_mean=mean(sa_data(:,4,:));

                    ym_table(inti_b,inti_d,inti_v,inti_a)=ym_mean;
                    dphi_table(inti_b,inti_d,inti_v,inti_a)=dphi_mean;
                    ay_table(inti_b,inti_d,inti_v,inti_a)=ay_mean;
                    ay_std_table(inti_b,inti_d,inti_v,inti_a)=ay_std;
                    sa_fl_table(inti_b,inti_d,inti_v,inti_a)=sa_fl_mean;
                    sa_fr_table(inti_b,inti_d,inti_v,inti_a)=sa_fr_mean;
                    sa_rl_table(inti_b,inti_d,inti_v,inti_a)=sa_rl_mean;
                    sa_rr_table(inti_b,inti_d,inti_v,inti_a)=sa_rr_mean;
                    sa_f_table(inti_b,inti_d,inti_v,inti_a)=(sa_fl_mean+sa_fr_mean)/2;
                    sa_r_table(inti_b,inti_d,inti_v,inti_a)=(sa_rl_mean+sa_rr_mean)/2;
                    
                    %update waitbar
                    z = z+1;
                    f = waitbar(z/overall_length,f,"Running Simulation...");
                end
            end
            %update waitbar
            f = waitbar(z/overall_length,f,"Save Diagrams...");

            %flip ym_table for nlstm becaus beta sign conversion is the other way
            %around
            if sim_model==2 || sim_model==3
                ym_table(:,:,inti_v,inti_a)=ym_table(:,:,inti_v,inti_a).*-1;
            end

            %create and save YMD plots to specified directory
            plot_YMD
            baseFileName=['YMD_ax' num2str(ax_range(inti_a)) 'mps2_v' num2str(v_range(inti_v)) 'mps' ];
            fullFileName = fullfile(path2outputs, baseFileName);
            saveas(figure(1),fullFileName)
            close all

             %create and save stability and cotrol plots to specified directory
            stability_control_plots

            baseFileName=['stability_ax' num2str(ax_range(inti_a)) 'mps2_v' num2str(v_range(inti_v)) 'mps' ];
            fullFileName = fullfile(path2outputs, baseFileName);
            saveas(figure(1),fullFileName)
            
            baseFileName=['sideslip_damping_ax' num2str(ax_range(inti_a)) 'mps2_v' num2str(v_range(inti_v)) 'mps' ];
            fullFileName = fullfile(path2outputs, baseFileName);
            saveas(figure(2),fullFileName)
            
            baseFileName=['control_moment_ax' num2str(ax_range(inti_a)) 'mps2_v' num2str(v_range(inti_v)) 'mps' ];
            fullFileName = fullfile(path2outputs, baseFileName);
            saveas(figure(3),fullFileName)

            baseFileName=['control_force_ax' num2str(ax_range(inti_a)) 'mps2_v' num2str(v_range(inti_v)) 'mps' ];
            fullFileName = fullfile(path2outputs, baseFileName);
            saveas(figure(4),fullFileName)
            close all

            calculate_KPI

           %create Slip angle delta plot
            plot_SA_delta
            baseFileName=['SA_delta_ax' num2str(ax_range(inti_a)) 'mps2_v' num2str(v_range(inti_v)) 'mps' ];
            fullFileName = fullfile(path2outputs, baseFileName);
            saveas(figure(1),fullFileName)
            close all

            %extract data for 3D plot
            calc_3d_v_YMD
            
        end
        %update waitbar
        f = waitbar(z/overall_length,f,"Rebuilding Model...");
        
        %Disable Fast Restart to force Rebuild
        set_param('ymd','FastRestart','off')
    end
    
    %save 3D YMD
    %loop for long. accel.
    for inti_a=1:max(size(ax_range))
        plot_3d_v_YMD
        baseFileName=['3D_velocity_YMD_ax' num2str(ax_range(inti_a)) 'mps2'];
        fullFileName = fullfile(path2outputs, baseFileName);
        saveas(figure(1),fullFileName)
        close all
    end
    
    %save output to .mat
    baseFileName='output.mat';
    fullFileName = fullfile(path2outputs, baseFileName);
    save(fullFileName,'ay_table','ym_table','banking','tv_factor','brake_balance',...
        'sim_model','delta_range','beta_range','v_range','ax_range','kpi','kpi_descr',...
        'kpi_unit','control_force_table','control_moment_table','sideslip_damping_table',...
        'stability_table')

    %save data dictionary file 
    baseFileName='DataDictionary.mat';
    fullFileName = fullfile(path2outputs, baseFileName);
    exportToFile(DataSect, fullFileName);
    
    set_param('ymd','SimulationMode','normal')
    %Enable all logging
    for x=1:length(pH)
       set_param(pH(x), 'DataLogging', 'on');
    end
    disp('Finished!')

    %close waitbar
    close(f)
