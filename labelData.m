% LABELDATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Skeletal Joints of the human body (Object Handovers Only)
%    TORSO (1)
%    NECK (2)
%    RIGHT_SHOULDER (3)
%    RIGHT_ELBOW (4)
%    RIGHT_HAND (5)
%    LEFT_SHOULDER (6)
%    LEFT_ELBOW (7)
%    LEFT_HAND (8)
%    HEAD (9)
%
%   @utor: Calvo, Natalia
%   nata.calvob@hotmail.com
%   Mater Thesis: Towards natural object human-robot handover
%   University of Genoa, Genoa, Italy
%   2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;  
workspace;  % Make sure the workspace panel is showing.
format longg;
format compact;

sample_time = 0.1; 
path_files = '/home/natycalvob/catkin_ws/src/thesis/src/Data_Coded/AllData';
general_path = fullfile(path_files, '*.csv');
d = dir(general_path);
numFiles = length(d);

cd AllData
for j=1:numFiles
    filename = d(j).name;
    NewTable = readtable(filename, 'Delimiter', ',');
    timeDuration(j) = size(NewTable,1)/10;
    namesFiles{j,1} = filename;
    
    % Total time per DataSet
    totalTime = array2table((sample_time:sample_time:(size(NewTable,1)/10))',...
        'VariableNames',{'Time'});
    
    % Remove the topics we are not interested in 
    onlyData = removevars(NewTable,{'rosbagTimestamp',... 
                         'left_foot_x','left_foot_y','left_foot_z',...
                         'left_knee_x','left_knee_y','left_knee_z',...
                         'left_hip_x','left_hip_y','left_hip_z',...
                         'right_foot_x','right_foot_y','right_foot_z',...
                         'right_knee_x','right_knee_y','right_knee_z',...
                         'right_hip_x','right_hip_y','right_hip_z'});
    
    
    % Data set with the sample time                 
    sampleData = [totalTime onlyData];
    % sampleData_All(j,:) = {filename, [sampleData]};
    
    %% Adjency Information
    [totalT, totalD] = size(sampleData);
    XT = zeros(totalT,2);
    YT = zeros(totalT,2);
    ZT = zeros(totalT,2);
    XL = zeros(totalT,3);
    YL = zeros(totalT,3);
    ZL = zeros(totalT,3);
    XR = zeros(totalT,3);
    YR = zeros(totalT,3);
    ZR = zeros(totalT,3);
    
    for i=1:totalT
        % RooJoints (1-2)
        XT(i,:) = [sampleData{i,26}, sampleData{i,14}];
        YT(i,:) = [sampleData{i,27}, sampleData{i,15}];
        ZT(i,:) = [sampleData{i,28}, sampleData{i,16}];

        % Left-hand Trajectory (3-4-5)
        XL(i,:) = [sampleData{i,23}, sampleData{i,17}, sampleData{i,20}];
        YL(i,:) = [sampleData{i,24}, sampleData{i,18}, sampleData{i,21}];
        ZL(i,:) = [sampleData{i,25}, sampleData{i,19}, sampleData{i,22}];
        
        % Right-hand Trajectory (6-7-8)
        XR(i,:) = [sampleData{i,11}, sampleData{i,5}, sampleData{i,8}];
        YR(i,:) = [sampleData{i,12}, sampleData{i,6}, sampleData{i,9}];
        ZR(i,:) = [sampleData{i,13}, sampleData{i,7}, sampleData{i,10}];
    end
    
    rootTable = array2table([XT YT ZT]);
    rootTable.Properties.VariableNames = {'torso_x' 'neck_x' ...
                                          'torso_y' 'neck_y' ...
                                          'torso_z' 'neck_z'};
    leftTable = array2table([XL YL ZL]);
    leftTable.Properties.VariableNames = {'left_shoulder_x' 'left_elbow_x' 'left_hand_x' ...
                                          'left_shoulder_y' 'left_elbow_y' 'left_hand_y' ...
                                          'left_shoulder_z' 'left_elbow_z' 'left_hand_z'};
    rightTable = array2table([XR YR ZR]);
    rightTable.Properties.VariableNames = {'right_shoulder_x' 'right_elbow_x' 'right_hand_x' ...
                                           'right_shoulder_y' 'right_elbow_y' 'right_hand_y' ...
                                           'right_shoulder_z' 'right_elbow_z' 'right_hand_z'};

    adjencyData(j,:) = {filename, timeDuration(j), rootTable, leftTable, rightTable};
    
end


%% This is the table with the time duration of each trial
GeneralTable = [cell2table(namesFiles) array2table(timeDuration')];
GeneralTable.Properties.VariableNames = {'File' 'Time'};
meanTimeDuration = mean(GeneralTable{:,2});
stdTimeDuration = std(GeneralTable{:,2});

overMean = timeDuration'>meanTimeDuration;
x1 = sum(overMean(:) == 1);
fixedTime = round(meanTimeDuration * 1.30, 1);
overfixedTime = timeDuration' > fixedTime;
x2 = sum(overfixedTime(:) == 1);

% Fixed Time = 5.6s
goodData = GeneralTable;
toDelete = goodData.Time > fixedTime;
adjencyData(toDelete,:) = []; 


%% Zero Padding 
% Maximun lenght is equal to fixedtime*10
MaxNumRow = fixedTime*10;
numGoodData = size(adjencyData,1);

for i=1:numGoodData
    [NumRow, NumCol] = size(adjencyData{i,3});
    if NumRow <= MaxNumRow
        values2Pad = MaxNumRow - NumRow;
        valuesroot = table2array(adjencyData{i,3});
        valuesleft = table2array(adjencyData{i,4});
        valuesright = table2array(adjencyData{i,5});
        
        % This is the data split into three sections
        padvaluesroot = array2table([valuesroot ; zeros(values2Pad,NumCol)]);
        padvaluesroot.Properties.VariableNames = {'torso_x' 'neck_x' ...
                                                  'torso_y' 'neck_y' ...
                                                  'torso_z' 'neck_z'};
                                             
        padvaluesleft = array2table([valuesleft ; zeros(values2Pad,NumCol+3)]);
        padvaluesleft.Properties.VariableNames = {'left_shoulder_x' 'left_elbow_x' 'left_hand_x' ...
                                                  'left_shoulder_y' 'left_elbow_y' 'left_hand_y' ...
                                                  'left_shoulder_z' 'left_elbow_z' 'left_hand_z'};

        padvaluesright = array2table([valuesright ; zeros(values2Pad,NumCol+3)]);
        padvaluesright.Properties.VariableNames = {'right_shoulder_x' 'right_elbow_x' 'right_hand_x' ...
                                                   'right_shoulder_y' 'right_elbow_y' 'right_hand_y' ...
                                                   'right_shoulder_z' 'right_elbow_z' 'right_hand_z'};
        % This is the merged Data                                       
        mergeData = array2table([valuesroot, valuesleft, valuesright ; zeros(values2Pad,24)]);
        mergeData.Properties.VariableNames = {'torso_x' 'neck_x' ...
                                               'torso_y' 'neck_y' ...
                                               'torso_z' 'neck_z' ...
                                               'left_shoulder_x' 'left_elbow_x' 'left_hand_x' ...
                                               'left_shoulder_y' 'left_elbow_y' 'left_hand_y' ...
                                               'left_shoulder_z' 'left_elbow_z' 'left_hand_z' ...
                                               'right_shoulder_x' 'right_elbow_x' 'right_hand_x' ...
                                               'right_shoulder_y' 'right_elbow_y' 'right_hand_y' ...
                                               'right_shoulder_z' 'right_elbow_z' 'right_hand_z'};

    end
    fullpaddingData(i,:) = {adjencyData{i,1}, padvaluesroot, padvaluesleft, padvaluesright};
    sortmergeData = mergeData(:,sort(mergeData.Properties.VariableNames));
    HandoverData(i,:) = {adjencyData{i,1}, sortmergeData};
    
end
topicNames = sortmergeData.Properties.VariableNames;
%% Create MergedData per Topic 
% datasetTopic(HandoverData, topicNames); 

%% Generate the Trajectories 
% Plot a sample of the elbow
figure, plot3(valuesleft(:,2),valuesleft(:,5),valuesleft(:,8), '*r')
hold on
plot3(valuesright(:,2),valuesright(:,5),valuesright(:,8), 'o')
hold off
legend('Left - Trajectory','Right - Trajectory');
title('Trajectories')

%% Plot random trajectories, Left and Right
r4 = randperm(numGoodData,10);
for i=1:size(r4,2)
    figure;
    subplot(3,2,1);
    x = (sample_time:sample_time:size(adjencyData{r4(i),4},1)/10);
    yr = table2array(adjencyData{r4(i),5}(:,1));
    plot(x,yr, 'r')
    hold on
    yr = table2array(adjencyData{r4(i),5}(:,4));
    plot(x,yr, 'b')
    yr = table2array(adjencyData{r4(i),5}(:,7));
    plot(x,yr, 'g')
    title('Right Shoulder Trajectories')
    legend('x position','y position','z position')
    hold off
    
    subplot(3,2,2);
    yl = table2array(adjencyData{r4(i),4}(:,1));
    plot(x,yl, 'r')
    hold on
    yl = table2array(adjencyData{r4(i),4}(:,4));
    plot(x,yl, 'b')
    yl = table2array(adjencyData{r4(i),4}(:,7));
    plot(x,yl, 'g')
    title('Left Shoulder Trajectories')
    legend('x position','y position','z position')
    hold off
    
    subplot(3,2,3);
    yr = table2array(adjencyData{r4(i),5}(:,2));
    plot(x,yr, 'r')
    hold on
    yr = table2array(adjencyData{r4(i),5}(:,5));
    plot(x,yr, 'b')
    yr = table2array(adjencyData{r4(i),5}(:,8));
    plot(x,yr, 'g')
    title('Right Elbow Trajectories')
    legend('x position','y position','z position')
    hold off
    
    subplot(3,2,4);
    yl = table2array(adjencyData{r4(i),4}(:,2));
    plot(x,yl, 'r')
    hold on
    yl = table2array(adjencyData{r4(i),4}(:,5));
    plot(x,yl, 'b')
    yl = table2array(adjencyData{r4(i),4}(:,8));
    plot(x,yl, 'g')
    title('Left Elbow Trajectories')
    legend('x position','y position','z position')
    hold off
    
    subplot(3,2,5);
    yr = table2array(adjencyData{r4(i),5}(:,3));
    plot(x,yr, 'r')
    hold on
    yr = table2array(adjencyData{r4(i),5}(:,6));
    plot(x,yr, 'b')
    yr = table2array(adjencyData{r4(i),5}(:,9));
    plot(x,yr, 'g')
    title('Right Hand Trajectories')
    legend('x position','y position','z position')
    hold off
    
    subplot(3,2,6);
    yl = table2array(adjencyData{r4(i),4}(:,3));
    plot(x,yl, 'r')
    hold on
    yl = table2array(adjencyData{r4(i),4}(:,6));
    plot(x,yl, 'b')
    yl = table2array(adjencyData{r4(i),4}(:,9));
    plot(x,yl, 'g')
    title('Left Elbow Trajectories')
    legend('x position','y position','z position')
    hold off
end


