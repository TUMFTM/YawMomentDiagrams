%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme File or the Quick Guide at the start
%of the 'create_YMD album script'

%%
%get size of solution matrix
sz=size(ay_table);

%run script to enhance plot readability
data_table_clean 

intr=1;
intc=1;
   
    %plot constant beta lines
    for intr=1:sz(1)
        b(intr)=plot(ay_table(intr,:,inti_v,inti_a,inti_ba).*clean_table(intr,:,inti_v,inti_a,inti_ba),ym_table(intr,:,inti_v,inti_a,inti_ba).*clean_table(intr,:,inti_v,inti_a,inti_ba),'b');
        %text(b(intr).XData(round(sz(2)/2)),b(intr).YData(round(sz(2)/2)),strcat('\leftarrow','beta:',num2str(beta_range(intr)),'rad'));
        if intr==1
            hold on
        end
    end

    %plot constant delta lines
    for intc=1:sz(2)
       d(intc)=plot(ay_table(:,intc,inti_v,inti_a,inti_ba).*clean_table(:,intc,inti_v,inti_a,inti_ba),ym_table(:,intc,inti_v,inti_a,inti_ba).*clean_table(:,intc,inti_v,inti_a,inti_ba),'r');
       %text(d(intc).XData(round(sz(1)/2)),d(intc).YData(round(sz(1)/2)),strcat('\leftarrow','delta:',num2str(delta_range(intc)),'rad'));
       if intc==1
            hold on
       end
    end
    

xlabel('lateral acceleration [m/s^{2}]')
ylabel('yawmoment [Nm]')
grid on
legend([b(1),d(1)],'constant beta','constant delta')
title(['Yaw Moment Diagram: ', num2str(v), 'mps, ', num2str(ax), 'mps2, banking', num2str(abs(banking)), 'deg'])
xlim([-x_lim, x_lim])
ylim([-y_lim, y_lim])