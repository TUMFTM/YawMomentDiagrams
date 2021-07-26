%This is a subscript of 'plot_KPI' and gets called during its
%execution. Please refer to the Readme File or the Quick Guide at the start
%of the 'plot_KPI' script

%%
%Split directory path by '\'
dir_split=regexp(dir,'\','split');

%extract last folder name
for kpi_int_d=1:max(size(dir))
dir_name(kpi_int_d)=dir_split{1,kpi_int_d}(max(size(dir_split{1,kpi_int_d})));
end

%load data from first directory
load(char(fullfile(dir(1), 'output.mat')));
kpi_dir(:,:,:,:,1)=kpi;

%load data from following directories
for kpi_int_d=2:max(size(dir))
    load(char(fullfile(dir(kpi_int_d), 'output.mat')),'kpi');
    kpi_dir(:,:,:,:,kpi_int_d)=kpi;
end