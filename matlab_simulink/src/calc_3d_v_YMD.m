%This is a subscript of 'create_YMD_albums' and gets called during its
%execution. Please refer to the Readme file or the Quick Guide at the start
%of the 'create_YMD album' script

%%

%creates 3D plot mit outer edges of YMD wit velocity as third axis
intr=1;
intc=1;

%find point in east
[intr,intc]=find(ay_table(:,:,inti_v,inti_a)==max(max(ay_table(:,:,inti_v,inti_a))));
ay_table_v(1,inti_v,inti_a)=ay_table(intr,intc,inti_v,inti_a);
ay_table_v(5,inti_v,inti_a)=ay_table(intr,intc,inti_v,inti_a);
ym_table_v(1,inti_v,inti_a)=ym_table(intr,intc,inti_v,inti_a);
ym_table_v(5,inti_v,inti_a)=ym_table(intr,intc,inti_v,inti_a);

%find point in west
[intr,intc]=find(ay_table(:,:,inti_v,inti_a)==min(min(ay_table(:,:,inti_v,inti_a))));
ay_table_v(3,inti_v,inti_a)=ay_table(intr,intc,inti_v,inti_a);
ym_table_v(3,inti_v,inti_a)=ym_table(intr,intc,inti_v,inti_a);

%find point in north
[intr,intc]=find(ym_table(:,:,inti_v,inti_a)==max(max(ym_table(:,:,inti_v,inti_a))));
ay_table_v(4,inti_v,inti_a)=ay_table(intr,intc,inti_v,inti_a);
ym_table_v(4,inti_v,inti_a)=ym_table(intr,intc,inti_v,inti_a);

%find point in south
[intr,intc]=find(ym_table(:,:,inti_v,inti_a)==min(min(ym_table(:,:,inti_v,inti_a))));
ay_table_v(2,inti_v,inti_a)=ay_table(intr,intc,inti_v,inti_a);
ym_table_v(2,inti_v,inti_a)=ym_table(intr,intc,inti_v,inti_a);




