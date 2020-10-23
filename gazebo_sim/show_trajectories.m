%%

close all
clear all
clc

%%
all_files = dir('telemetry/*.log');

for f = 1:length(all_files)
    %X = importdata('telemetry/ex1_2020_10_22_22_12_55.log',';',1);
    X = importdata([all_files(f).folder '/' all_files(f).name], ';', 1);

    data = X.data;

    i=1;
    t = data(:,i)-data(1,i); i=i+1;
    x_goal = data(:, i); i=i+1;
    y_goal = data(:, i); i=i+1;
    x_waypoint = data(:,i);i=i+1;
    y_waypoint = data(:,i);i=i+1;
    teta_waypoint = data(:,i);i=i+1;
    x_groundtruth = data(:,i);i=i+1;
    y_groundtruth = data(:,i);i=i+1;
    teta_groundtruth = data(:,i);i=i+1;
    x_perception = data(:,i);i=i+1;
    y_perception = data(:,i);i=i+1;
    teta_perception = data(:,i);i=i+1;

    %%
    figure(1);
    plot(t, [x_groundtruth-x_perception, y_groundtruth-y_perception]);
    title('groud truth - estimation'); legend('x', 'y'); grid on;
    xlabel('t [sec]'); ylabel('\Delta [m]'); hold all

    figure(2);
    plot(x_waypoint, y_waypoint);
    title('global planner commands'); grid on;
    xlabel('X [m]'); ylabel('Y [m]'); hold all; axis equal
    
    figure(3);
    plot( x_groundtruth, y_groundtruth);
    title('groud truth'); grid on;
    xlabel('X [m]'); ylabel('Y [m]'); hold all; axis equal

end