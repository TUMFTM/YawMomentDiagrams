%This is a subscript of 'main_calcYMD' and gets called during its
%execution. Please refer to the Readme file or the Quick Guide at the start
%of the 'create_YMD album' script

% if the loop in 'main_calcYMD' reaches the end of a velocity loop this
% if-condition with enclosed loop generates the 3D-velocity diagram
    for k=1:max(size(v_range))
    b(k)=plot(ay_table_v(:,k,inti_a,inti_ba),ym_table_v(:,k,inti_a,inti_ba),'DisplayName',strcat(num2str(v_range(k)),'mps'));

    if k==1
        hold on
    end
    end
xlabel('lateral acceleration [m/s^{2}]')
ylabel('yawmoment [Nm]')
grid on
title(['3D Yaw Moment Diagram @ a_x=', num2str(ax), 'mps2, banking=', num2str(abs(banking)), 'deg'])
legend
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])



