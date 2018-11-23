%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This scritp is to evaluate each dataset and visualize the trajectory 
%   of the body joints 
%
%   @utor: Calvo, Natalia
%   nata.calvob@hotmail.com
%   Mater Thesis: Towards natural object human-robot handover
%   University of Genoa, Genoa, Italy
%   2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Modify the following datasets to optimize the full data 
close all
filename1 = ('R_U21_C1_O1_2.csv');

longData_Table = readtable(filename1, 'Delimiter', ',');
sample_time = 0.1

% Total time per DataSet
totalTime = array2table((sample_time:sample_time:(size(longData_Table,1)/10))',...
        'VariableNames',{'Time'});
    
% Remove the topics we are not interested in 
onlyLongData = removevars(longData_Table,{'rosbagTimestamp',...
    'left_foot_x','left_foot_y','left_foot_z',...
    'left_knee_x','left_knee_y','left_knee_z',...
    'left_hip_x','left_hip_y','left_hip_z',...
    'right_foot_x','right_foot_y','right_foot_z',...
    'right_knee_x','right_knee_y','right_knee_z',...
    'right_hip_x','right_hip_y','right_hip_z'});

% Data set with the sample time                 
sampleLongData = [totalTime onlyLongData];
    

%% Adjency Information
[totalT, totalD] = size(sampleLongData);

figure;

for i=2:totalT
    clf(1)
    hold on
    % Left-hand Trajectory (1-2-3-4-5)
    XL = [sampleLongData{i,26}, sampleLongData{i,14}, sampleLongData{i,23}, sampleLongData{i,17}, sampleLongData{i,20}];
    YL = [sampleLongData{i,27}, sampleLongData{i,15}, sampleLongData{i,24}, sampleLongData{i,18}, sampleLongData{i,21}];
    ZL = [sampleLongData{i,28}, sampleLongData{i,16}, sampleLongData{i,25}, sampleLongData{i,19}, sampleLongData{i,22}];
    plot3(XL,YL,ZL,'black')
    
    % Right-hand Trajectory (1-2-6-7-8)
    XR = [sampleLongData{i,26}, sampleLongData{i,14}, sampleLongData{i,11}, sampleLongData{i,5}, sampleLongData{i,8}];
    YR= [sampleLongData{i,27}, sampleLongData{i,15}, sampleLongData{i,12}, sampleLongData{i,6}, sampleLongData{i,9}];
    ZR= [sampleLongData{i,28}, sampleLongData{i,16}, sampleLongData{i,13}, sampleLongData{i,7}, sampleLongData{i,10}];
    plot3(XR,YR,ZR,'red')
    
    XS = [sampleLongData{i,14}];
    YS = [sampleLongData{i,15}];
    ZS = [sampleLongData{i,16}];
    plot3(XS,YS,ZS,'o');
    pause(sample_time);
end


%% Generate the Trajectories 
% Plot a sample of the elbow
sampleLongData1 = table2array(smoothdata(sampleLongData));
figure, 
plot3(sampleLongData1(:,5),sampleLongData1(:,6),sampleLongData1(:,7), 'o')
hold on
plot3(sampleLongData1(:,17),sampleLongData1(:,18),sampleLongData1(:,19), '*r')
hold off
legend('Right - Trajectory','Left - Trajectory');
title('Trajectories')

%% Plot random trajectories, Left and Right

% Shoulder vs Elbow
figure, 
x = (sample_time:sample_time:size(sampleLongData,1)/10)';
plot3(x,sampleLongData1(:,11),sampleLongData1(:,12), 'r')
plot3(x,sampleLongData1(:,5),sampleLongData1(:,6), 'b')
plot3(x,sampleLongData1(:,8),sampleLongData1(:,9), 'g')
hold off
xlabel('Time(s)')
ylabel('X(cm)')
zlabel('Y(cm)')
legend('Shoulder - Trajectory','Elbow - Trajectory', 'Hand - Trajectory');
title('Right Arm Trajectories')

figure,
hold on
plot(sampleLongData1(:,12),sampleLongData1(:,13), 'r')
plot(sampleLongData1(:,6),sampleLongData1(:,7), 'b')
plot(sampleLongData1(:,9),sampleLongData1(:,10), 'g')
hold off
xlabel('Y(cm)')
ylabel('Z(cm)')
legend('Shoulder - Trajectory','Elbow - Trajectory', 'Hand - Trajectory');
title('Right Arm Trajectories')

figure,
hold on
plot(sampleLongData1(:,5),sampleLongData1(:,7), 'b')
plot(sampleLongData1(:,8),sampleLongData1(:,10), 'g')
hold off
xlabel('X(cm)')
ylabel('Z(cm)')
legend('Elbow - Trajectory', 'Hand - Trajectory');
title('Right Arm Trajectories')

figure,
hold on
plot(sampleLongData1(:,5),sampleLongData1(:,6), 'b')
plot(sampleLongData1(:,8),sampleLongData1(:,9), 'g')
hold off
xlabel('X(cm)')
ylabel('Y(cm)')
legend('Elbow - Trajectory', 'Hand - Trajectory');
title('Right Arm Trajectories')

figure,
hold on
plot(sampleLongData1(:,6),sampleLongData1(:,7), 'b')
plot(sampleLongData1(:,9),sampleLongData1(:,10), 'g')
hold off
xlabel('Y(cm)')
ylabel('Z(cm)')
legend('Elbow - Trajectory', 'Hand - Trajectory');
title('Right Arm Trajectories')

%% Plots Position and Velocities
xvel = (sample_time:sample_time:(size(sampleLongData,1)-1)/10)';
figure;
subplot(4,2,1);
yr = sampleLongData1(:,11);
hold on
grid on
plot(x,yr, 'r')
yr = sampleLongData1(:,5);
plot(x,yr, 'b')
yr = sampleLongData1(:,8);
plot(x,yr, 'g')
title('Position in X-axis - Right Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('X(m)')
hold off

subplot(4,2,2);
yr = sampleLongData1(:,23);
plot(x,yr, 'r')
hold on
grid on
yr = sampleLongData1(:,17);
plot(x,yr, 'b')
yr = sampleLongData1(:,20);
plot(x,yr, 'g')
title('Position in X-axis - Left Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('X(m)')
hold off

subplot(4,2,3);
yr = sampleLongData1(:,12);
plot(x,yr, 'r')
hold on
grid on
yr = sampleLongData1(:,6);
plot(x,yr, 'b')
yr = sampleLongData1(:,9);
plot(x,yr, 'g')
title('Position in Y-axis - Right Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('Y(m)')
hold off


subplot(4,2,4);
yr = sampleLongData1(:,24);
plot(x,yr, 'r')
hold on
grid on
yr = sampleLongData1(:,18);
plot(x,yr, 'b')
yr = sampleLongData1(:,21);
plot(x,yr, 'g')
title('Position in Y-axis - Left Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('Y(m)')
hold off


subplot(4,2,5);
yr = sampleLongData1(:,13);
plot(x,yr, 'r')
hold on
grid on
yr = sampleLongData1(:,7);
plot(x,yr, 'b')
yr = sampleLongData1(:,10);
plot(x,yr, 'g')
title('Position in Z-axis - Right Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('Z(m)')
hold off


subplot(4,2,6);
yr = sampleLongData1(:,25);
plot(x,yr, 'r')
hold on
grid on
yr = sampleLongData1(:,19);
plot(x,yr, 'b')
yr = sampleLongData1(:,22);
plot(x,yr, 'g')
title('Position in Z-axis - Left Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('Z(m)')
hold off

subplot(4,2,7);
vx_s = diff(sampleLongData1(:,11))/sample_time;
vy_s = diff(sampleLongData1(:,12))/sample_time;
vz_s = diff(sampleLongData1(:,13))/sample_time;
vx_e = diff(sampleLongData1(:,5))/sample_time;
vy_e = diff(sampleLongData1(:,6))/sample_time;
vz_e = diff(sampleLongData1(:,7))/sample_time;
vx_h = diff(sampleLongData1(:,8))/sample_time;
vy_h = diff(sampleLongData1(:,9))/sample_time;
vz_h = diff(sampleLongData1(:,10))/sample_time;
yx = smooth(sqrt(vx_s.^2 + vy_s.^2 + vz_s.^2));
[Va, Id] = max(yx);
formatSpec = 'Shoulder, the max Velocity is %4.2f m/s at %4.2f s\n';
fprintf(formatSpec,Va,Id/10);
plot(xvel,yx, 'r')
hold on
grid on
yy = smooth(sqrt(vx_e.^2 + vy_e.^2 + vz_e.^2));
[Va, Id] = max(yy);
formatSpec = 'Elbow, the max Velocity is %4.2f m/s at %4.2f s\n';
fprintf(formatSpec,Va,Id/10);
plot(xvel,yy, 'b')
yz = smooth(sqrt(vx_h.^2 + vy_h.^2 + vz_h.^2));
[Va, Id] = max(yz);
formatSpec = 'Hand, the max Velocity is %4.2f m/s at %4.2f s\n';
fprintf(formatSpec,Va,Id/10);
plot(xvel,yz, 'g')
title('Linear Velocity - Right Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('V(m/s)')
hold off

subplot(4,2,8);
vx_s = diff(sampleLongData1(:,23))/sample_time;
vy_s = diff(sampleLongData1(:,24))/sample_time;
vz_s = diff(sampleLongData1(:,25))/sample_time;
vx_e = diff(sampleLongData1(:,17))/sample_time;
vy_e = diff(sampleLongData1(:,18))/sample_time;
vz_e = diff(sampleLongData1(:,19))/sample_time;
vx_h = diff(sampleLongData1(:,20))/sample_time;
vy_h = diff(sampleLongData1(:,21))/sample_time;
vz_h = diff(sampleLongData1(:,22))/sample_time;
yx = sqrt(vx_s.^2 + vy_s.^2 + vz_s.^2);
[Va, Id] = max(yx);
formatSpec = 'Shoulder, the max Velocity is %4.2f m/s at %4.2f s\n';
fprintf(formatSpec,Va,Id/10);
hold on
grid on
plot(xvel,yx, 'r')
yy = sqrt(vx_e.^2 + vy_e.^2 + vz_e.^2);
[Va, Id] = max(yy);
formatSpec = 'Elbow, the max Velocity is %4.2f m/s at %4.2f s\n';
fprintf(formatSpec,Va,Id/10);
plot(xvel,yy, 'b')
yz = sqrt(vx_h.^2 + vy_h.^2 + vz_h.^2);
[Va, Id] = max(yz);
formatSpec = 'Hand, the max Velocity is %4.2f m/s at %4.2f s\n';
fprintf(formatSpec,Va,Id/10);
yz = smooth(yz);
plot(xvel,yz, 'g')
title('Linear Velocity - Left Arm')
legend('Shoulder','Elbow','Hand')
xlabel('Time(s)')
ylabel('V(m/s)')
hold off

