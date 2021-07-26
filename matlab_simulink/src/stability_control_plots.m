%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme File or the Quick Guide at the start
%of the 'create_YMD album script'

%%
%get size of solution matrix
sz=size(ay_table);
beta_step=beta_range(2)-beta_range(1);
delta_step=delta_range(2)-delta_range(1);

%Calcutate stability matrix
for intc3=1:sz(2)
    for intr3=1:sz(1)
        if intr3==sz(1)
            stability_1(intr3, intc3, inti_v, inti_a)=NaN;
            sideslip_damping_1(intr3, intc3, inti_v, inti_a)=NaN;
            
        else
            stability_1(intr3, intc3, inti_v, inti_a)=(ym_table(intr3+1, intc3, inti_v, inti_a)...
            -ym_table(intr3, intc3, inti_v, inti_a))/beta_step/180*pi;
            sideslip_damping_1(intr3, intc3, inti_v, inti_a)=(ay_table(intr3+1, intc3, inti_v, inti_a)...
            -ay_table(intr3, intc3, inti_v, inti_a))/beta_step/180*pi;
        end
        
        if intr3==1
            stability_2(intr3, intc3, inti_v, inti_a)=NaN;
            sideslip_damping_2(intr3, intc3, inti_v, inti_a)=NaN;
            
        else
            stability_2(intr3, intc3, inti_v, inti_a)=(ym_table(intr3, intc3, inti_v, inti_a)...
            -ym_table(intr3-1, intc3, inti_v, inti_a))/beta_step/180*pi;
            sideslip_damping_2(intr3, intc3, inti_v, inti_a)=(ay_table(intr3, intc3, inti_v, inti_a)...
            -ay_table(intr3-1, intc3, inti_v, inti_a))/beta_step/180*pi;

        end
        
     end
end

stability_table=(stability_1+stability_2)./2;
sideslip_damping_table=(sideslip_damping_1+sideslip_damping_2)./2;


% %Calcutate control force and control moment matrix
for intr3=1:sz(1)
    for intc3=1:sz(2)
        if intc3==sz(2)
            control_moment_1(intr3, intc3, inti_v, inti_a)=NaN;
            control_force_1(intr3, intc3, inti_v, inti_a)=NaN;
            
        else
            control_moment_1(intr3, intc3, inti_v, inti_a)=(ym_table(intr3, intc3+1, inti_v, inti_a)...
            -ym_table(intr3, intc3, inti_v, inti_a))/delta_step/180*pi;
            control_force_1(intr3, intc3, inti_v, inti_a)=(ay_table(intr3, intc3+1, inti_v, inti_a)...
            -ay_table(intr3, intc3, inti_v, inti_a))/delta_step/180*pi;
           
        end
        
        if intc3==1
            control_moment_2(intr3, intc3, inti_v, inti_a)=NaN;
            control_force_2(intr3, intc3, inti_v, inti_a)=NaN;
            
        else
            control_moment_2(intr3, intc3, inti_v, inti_a)=(ym_table(intr3, intc3, inti_v, inti_a)...
            -ym_table(intr3, intc3-1, inti_v, inti_a))/delta_step/180*pi;
            control_force_2(intr3, intc3, inti_v, inti_a)=(ay_table(intr3, intc3, inti_v, inti_a)...
            -ay_table(intr3, intc3-1, inti_v, inti_a))/delta_step/180*pi;
        end
        
     end
end

control_moment_table(:,:,inti_v,inti_a)=(control_moment_1(:,:,inti_v,inti_a)+control_moment_2(:,:,inti_v,inti_a))./2;
control_force_table(:,:,inti_v,inti_a)=(control_force_1(:,:,inti_v,inti_a)+control_force_2(:,:,inti_v,inti_a))./2;


%plot stability diagram
figure(1)
[c,h]=contourf(ay_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    ym_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    stability_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),20,'ShowText','on');
h.LevelList=round(h.LevelList,0);
clabel(c,h, 'FontSize', 8);
xlabel('lateral acceleration [m/s^{2}]')
ylabel('yaw moment [Nm]')
grid on
title(['stability @ ', num2str(v), 'mps, ', num2str(ax), 'mps2'])
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])
cb = colorbar;
ylabel(cb, 'yaw moment per vehicle slip angle [Nm/deg]')

%plot sideslip damping diagram
figure(2)
[c,h]=contourf(ay_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    ym_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    sideslip_damping_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),20,'ShowText','on');
h.LevelList=round(h.LevelList,0);
clabel(c,h, 'FontSize', 8);
xlabel('lateral acceleration [m/s^{2}]')
ylabel('yaw moment [Nm]')
grid on
title(['sideslip damping @ ', num2str(v), 'mps, ', num2str(ax), 'mps2'])
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])
cb = colorbar;
ylabel(cb, 'lateral acceleration per vehicle slip angle [Nm/deg]')


%plot control moment diagram
figure(3)
[c,h]=contourf(ay_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    ym_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    control_moment_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),20);
h.LevelList=round(h.LevelList,0);
clabel(c,h, 'FontSize', 8);
xlabel('lateral acceleration [m/s^{2}]')
ylabel('yaw moment [Nm]')
grid on
title(['control moment @ ', num2str(v), 'mps, ', num2str(ax), 'mps2'])
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])
cb = colorbar;
ylabel(cb, 'yaw moment per steering angle [Nm/deg]')

%plot control force diagram
figure(4)
[c,h]=contourf(ay_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    ym_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    control_force_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),20);
h.LevelList=round(h.LevelList,1);
clabel(c,h, 'FontSize', 8);
xlabel('lateral acceleration [m/s^{2}]')
ylabel('yaw moment [Nm]')
grid on
title(['control force @ ', num2str(v), 'mps, ', num2str(ax), 'mps2'])
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])
cb = colorbar;
ylabel(cb, 'lateral acceleration per steering angle [m/s^2/deg]')

