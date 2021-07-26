%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme File or the Quick Guide at the start
%of the 'create_YMD album script'

%%
%KPI descriptions

%General KPI 
kpi_descr(1)={'Mz max'}; 
kpi_unit(1)={'Maximum Yaw Moment [Nm]'};

% for the following KPI 'lim' corresponds to the point of maximum lateral acceleration 
kpi_descr(2)={'ay@lim'}; %
kpi_unit(2)={'Maximum lateral acceleration [m/s^2]'};
kpi_descr(3)={'Mz@lim'}; %
kpi_unit(3)={'Yaw Moment @ lim [Nm]'};
kpi_descr(4)={'SAf@lim'}; %
kpi_unit(4)={'Slip angle front @ lim [deg]'};
kpi_descr(5)={'SAr@lim'}; %
kpi_unit(5)={'Slip angle rear @ lim [deg]'};
kpi_descr(6)={'SAdiff@lim'}; %
kpi_unit(6)={'Slip angle difference @ lim [deg]'};
kpi_descr(7)={[char(946) '@lim']}; %beta @ lim [deg]
kpi_unit(7)={'beta @ lim [deg]'};
kpi_descr(8)={[char(948) '@lim']}; %
kpi_unit(8)={'delta @ lim [deg]'};
kpi_descr(9)={['dMz/d' char(946) '@lim']}; %
kpi_unit(9)={'stability @ lim [Nm/deg]'};

% for the following KPI 'trim' corresponds to the point of maximum lateral
% acceleration archivable in steady state (trimmed condition) --> yawmoment=0Nm
kpi_descr(10)={'ay@trim'}; %
kpi_unit(10)={'maximum lateral acceleration in trimmed condition [m/s^2]'};
kpi_descr(11)={'SAf@trim'}; %
kpi_unit(11)={'Slip angle front @ trim [deg]'};
kpi_descr(12)={'SAr@trim'}; %
kpi_unit(12)={'Slip angle rear @ trim [deg]'};
kpi_descr(13)={'SAdiff@trim'}; %
kpi_unit(13)={'Slip angle difference @ trim [deg]'};
kpi_descr(14)={[char(946) '@trim']}; 
kpi_unit(14)={'beta @ trim [deg]'};
kpi_descr(15)={[char(948) '@trim']}; 
kpi_unit(15)={'delta @ trim [deg]'};
kpi_descr(16)={['dMz/d' char(946) '@trim']}; %
kpi_unit(16)={'stability @ trim [Nm/deg]'};

% for the following KPI 'stright' corresponds to delta=0 and beta=0, e.g.
% driving in a straight line
kpi_descr(17)={['dMz/d' char(948) '@straight']}; 
kpi_unit(17)={'control moment @ straight [Nm/deg]'};
kpi_descr(18)={['dMz/d' char(946) '@straight']}; %
kpi_unit(18)={'stability @ straight [Nm/deg]'};
kpi_descr(19)={['day/d' char(948) '@straight']}; %
kpi_unit(19)={'control force @ straight [ay/deg]'};

%%
%KPI calculations

%1:calc MZ_max
kpi(1,1,inti_v,inti_a)=max(ym_table(:,:,inti_v,inti_a),[],'all');
kpi(2,1,inti_v,inti_a)=min(ym_table(:,:,inti_v,inti_a),[],'all');

%2:calc ay_lim
kpi(1,2,inti_v,inti_a)=max(ay_table(:,:,inti_v,inti_a),[],'all');
kpi(2,2,inti_v,inti_a)=min(ay_table(:,:,inti_v,inti_a),[],'all');
[intr1, intc1]=find(ay_table(:,:,inti_v,inti_a)==kpi(1,2,inti_v,inti_a));
[intr2, intc2]=find(ay_table(:,:,inti_v,inti_a)==kpi(2,2,inti_v,inti_a));

%3: calc Mz@lim
kpi(1,3,inti_v,inti_a)=ym_table(intr1, intc1, inti_v,inti_a);
kpi(2,3,inti_v,inti_a)=ym_table(intr2, intc2, inti_v,inti_a);

%4: SAf@lim
kpi(1,4,inti_v,inti_a)=sa_f_table(intr1, intc1, inti_v,inti_a)/pi*180;
kpi(2,4,inti_v,inti_a)=sa_f_table(intr2, intc2, inti_v,inti_a)/pi*180;

%5: SAr@lim
kpi(1,5,inti_v,inti_a)=sa_r_table(intr1, intc1, inti_v,inti_a)/pi*180;
kpi(2,5,inti_v,inti_a)=sa_r_table(intr2, intc2, inti_v,inti_a)/pi*180;

%6: SAdiff@lim
kpi(1,6,inti_v,inti_a)=(sa_f_table(intr1, intc1, inti_v,inti_a)-sa_r_table(intr1, intc1, inti_v,inti_a))/pi*180;
kpi(2,6,inti_v,inti_a)=(sa_f_table(intr2, intc2, inti_v,inti_a)-sa_r_table(intr2, intc2, inti_v,inti_a))/pi*180;

%7: beta@lim
kpi(1,7,inti_v,inti_a)=beta_range(intr1)/pi*180;
kpi(2,7,inti_v,inti_a)=beta_range(intr2)/pi*180;

%8: delta@lim
kpi(1,8,inti_v,inti_a)=delta_range(intc1)/pi*180;
kpi(2,8,inti_v,inti_a)=delta_range(intc2)/pi*180;

%9: dMz/dbeta@lim
kpi(1,9,inti_v,inti_a)=stability_table(intr1,intc1,inti_v,inti_a);
kpi(2,9,inti_v,inti_a)=stability_table(intr2,intc2,inti_v,inti_a);

% find "trim" point. moves along constant delta lines and gives back the
% point with highest lateral acceleration and one beta step after crossing
% the x axis (Mz=0)
ay_last=0;
ay_curr=0;
ym_last=0;
ym_curr=0;

for kpi_inti_d=1:max(size(delta_range))
  ym_curr=ym_table(max(size(beta_range)),kpi_inti_d,inti_v,inti_a);
  ym_last=ym_curr;
  for kpi_inti_b=flip(1:(max(size(beta_range))-1))
      ym_last=ym_curr;
      ym_curr=ym_table(kpi_inti_b,kpi_inti_d,inti_v,inti_a)*clean_table(kpi_inti_b,kpi_inti_d,inti_v,inti_a);
      
      if ym_curr*ym_last <0
          ay_curr=ay_table(kpi_inti_b,kpi_inti_d,inti_v,inti_a);
      end
      
      if ay_curr>ay_last
          ay_last=ay_curr;
          intr1=kpi_inti_b;
          intc1=kpi_inti_d;
      end
  end
end

ay_last=0;
ay_curr=0;
ym_last=0;
ym_curr=0;

for kpi_inti_d=1:max(size(delta_range))
  ym_curr=ym_table(max(size(beta_range)),kpi_inti_d,inti_v,inti_a)*clean_table(kpi_inti_b,kpi_inti_d,inti_v,inti_a);
  ym_last=ym_curr;
  for kpi_inti_b=2:max(size(beta_range))
      ym_last=ym_curr;
      ym_curr=ym_table(kpi_inti_b,kpi_inti_d,inti_v,inti_a);
      
      if ym_curr*ym_last <0
          ay_curr=ay_table(kpi_inti_b,kpi_inti_d,inti_v,inti_a);
      end
      
      if ay_curr<ay_last
          ay_last=ay_curr;
          intr2=kpi_inti_b;
          intc2=kpi_inti_d;
      end
  end
end


%10: ay@trim
trim_1_fac_over=abs(ym_table(intr1+1,intc1,inti_v,inti_a))/(abs(ym_table(intr1+1,intc1,inti_v,inti_a))+abs(ym_table(intr1,intc1,inti_v,inti_a)));
trim_1_fac_under=abs(ym_table(intr1,intc1,inti_v,inti_a))/(abs(ym_table(intr1+1,intc1,inti_v,inti_a))+abs(ym_table(intr1,intc1,inti_v,inti_a)));
kpi(1,10,inti_v,inti_a)=trim_1_fac_under*ay_table(intr1+1,intc1,inti_v,inti_a)+trim_1_fac_over*ay_table(intr1,intc1,inti_v,inti_a);

trim_2_fac_over=abs(ym_table(intr2-1,intc2,inti_v,inti_a))/(abs(ym_table(intr2-1,intc2,inti_v,inti_a))+abs(ym_table(intr2,intc2,inti_v,inti_a)));
trim_2_fac_under=abs(ym_table(intr2,intc2,inti_v,inti_a))/(abs(ym_table(intr2-1,intc2,inti_v,inti_a))+abs(ym_table(intr2,intc2,inti_v,inti_a)));
kpi(2,10,inti_v,inti_a)=trim_2_fac_under*ay_table(intr2-1,intc2,inti_v,inti_a)+trim_2_fac_over*ay_table(intr2,intc2,inti_v,inti_a);

%11: SAf@trim
kpi(1,11,inti_v,inti_a)=(trim_1_fac_under*sa_f_table(intr1+1, intc1, inti_v,inti_a)+trim_1_fac_over*sa_f_table(intr1, intc1, inti_v,inti_a))/pi*180;
kpi(2,11,inti_v,inti_a)=(trim_2_fac_under*sa_f_table(intr2-1, intc2, inti_v,inti_a)+trim_2_fac_over*sa_f_table(intr2, intc2, inti_v,inti_a))/pi*180;

%12: SAr@trim
kpi(1,12,inti_v,inti_a)=(trim_1_fac_under*sa_r_table(intr1+1, intc1, inti_v,inti_a)+trim_1_fac_over*sa_r_table(intr1, intc1, inti_v,inti_a))/pi*180;
kpi(2,12,inti_v,inti_a)=(trim_2_fac_under*sa_r_table(intr2-1, intc2, inti_v,inti_a)+trim_2_fac_over*sa_r_table(intr2, intc2, inti_v,inti_a))/pi*180;

%13: SAdiff@trim
kpi(1,13,inti_v,inti_a)=kpi(1,11,inti_v,inti_a)-kpi(1,12,inti_v,inti_a);
kpi(2,13,inti_v,inti_a)=kpi(2,11,inti_v,inti_a)-kpi(2,12,inti_v,inti_a);

%14: beta@trim
kpi(1,14,inti_v,inti_a)=(trim_1_fac_under*beta_range(intr1+1)+trim_1_fac_over*beta_range(intr1))/pi*180;
kpi(2,14,inti_v,inti_a)=(trim_2_fac_under*beta_range(intr2-1)+trim_2_fac_over*beta_range(intr2))/pi*180;

%15: delta@trim
kpi(1,15,inti_v,inti_a)=delta_range(intc1)/pi*180;
kpi(2,15,inti_v,inti_a)=delta_range(intc2)/pi*180;

%16: dMz/dbeta@trim
kpi(1,16,inti_v,inti_a)=trim_1_fac_under*stability_table(intr1+1,intc1,inti_v,inti_a)+trim_1_fac_over*stability_table(intr1,intc1,inti_v,inti_a);
kpi(2,16,inti_v,inti_a)=trim_2_fac_under*stability_table(intr2-1,intc2,inti_v,inti_a)+trim_2_fac_over*stability_table(intr2,intc2,inti_v,inti_a);

%find point of origin (0,0)
intr1=find(beta_range==min(abs(beta_range)));
intc1=find(delta_range==min(abs(delta_range)));

%17: dMz/ddelta@straight
kpi(1,17,inti_v,inti_a)=control_moment_table(intr1,intc1,inti_v,inti_a);

%18: dMz/dbeta@straight
kpi(1,18,inti_v,inti_a)=stability_table(intr1,intc1,inti_v,inti_a);

%19 day/ddelta@straight
kpi(1,19,inti_v,inti_a)=control_force_table(intr1,intc1,inti_v,inti_a);
