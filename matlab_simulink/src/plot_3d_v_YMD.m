%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme file or the Quick Guide at the start
%of the 'create_YMD album' script

% if the loop in 'create_YMD_album' reaches the end of a velocity loop this
% if-condition with enclosed loop generates the 3D-velocity diagram
    for k=1:max(size(v_range))
    b(k)=plot(ay_table_v(:,k,inti_a),ym_table_v(:,k,inti_a),'DisplayName',strcat(num2str(v_range(k)),'mps'));

    if k==1
        hold on
    end
    end
xlabel('lateral acceleration [m/s^{2}]')
ylabel('yawmoment [Nm]')
grid on
title(['3D Yaw Moment Diagram @ ', num2str(ax), 'mps2'])
legend
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])



