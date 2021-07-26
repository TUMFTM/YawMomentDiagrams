%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme File or the Quick Guide at the start
%of the 'create_YMD album script'

%%

%calc sa_delta
sa_delta_table(:,:,inti_v,inti_a)=sa_f_table(:,:,inti_v,inti_a)-sa_r_table(:,:,inti_v,inti_a);
sa_delta_table_plot(:,:,inti_v,inti_a)=sa_delta_table(:,:,inti_v,inti_a)/pi*180;

[c,h]=contourf(ay_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    ym_table(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a),...
    abs(sa_delta_table_plot(:,:,inti_v,inti_a).*clean_table(:,:,inti_v,inti_a)),20);
h.LevelList=round(h.LevelList,1);
clabel(c,h, 'FontSize', 8);

xlabel('lateral acceleration [m/s^{2}]')
ylabel('yawmoment [Nm]')
grid on
title(['absolute Slip Angle delta: ', num2str(v), 'mps, ', num2str(ax), 'mps2'])
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])
cb = colorbar;
ylabel(cb, 'abs slip angle difference [deg]')
