% MERGEDALL 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This Script takes a folder and its subfoders paths,
%   then it processes all the .CSV files on those folders and read them
%   Finally generates a single file CSV with the data we are interesting in
%
%   @utor: Calvo, Natalia
%   nata.calvob@hotmail.com
%   Mater Thesis: Towards natural object human-robot handover
%   University of Genoa, Genoa, Italy
%   2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;  
workspace;  % Make sure the workspace panel is showing.
format longg;
format compact;

% Define a starting folder.
start_path = '/home/natycalvob/catkin_ws/src/thesis/src/Data_Coded';

% Ask user to confirm or change.
topLevelFolder = uigetdir(start_path);
if topLevelFolder == 0
	return;
end
% Get list of all subfolders.
allSubFolders = genpath(topLevelFolder);
% Parse into a cell array.
remain = allSubFolders;
listOfFolderNames = {};
while true
	[singleSubFolder, remain] = strtok(remain, ':');
	if isempty(singleSubFolder)
		break;
    end
    listOfFolderNames = [listOfFolderNames singleSubFolder];
end
numberOfFolders = length(listOfFolderNames)

% Process all CSV files in those folders.
for k = 1 : numberOfFolders
	% Get this folder and print it out.
	thisFolder = listOfFolderNames{k};
	fprintf('Processing folder %s\n', thisFolder);
	% Get CSV files
	filePattern = sprintf('%s/*.csv', thisFolder);
    % Now we have a list of all files in this folder.
	baseFileNames = dir(filePattern);
	numberOfCVSFiles = length(baseFileNames);
    data=cell(1,numberOfCVSFiles);     % preallocate a cell array to hold results
    data = cell2table(data);
    
    %% Merge all the data in a table 
	if numberOfCVSFiles >= 1
		for f = 1 : numberOfCVSFiles
            fullFileName = fullfile(thisFolder, baseFileNames(f).name);
            T = readtable(fullFileName);  % read each file
            if f == 1
                FullT = T;
            else
                T = removevars(T,{'rosbagTimestamp'});
                FullT = [FullT T];
            end
            numMsgSubTopic = size(FullT,1);
            sizeTableCol = size(FullT,2);
            
            %% rosbagTimesample to Time Duration 
            if sizeTableCol == 46
                numMsgSubTopic = size(FullT, 1);% Number of Message for each subtopic. (e.i. head_x)
                dateTimeMS = cell(numMsgSubTopic,1);
                t = cell(numMsgSubTopic,1);
                for i=1:numMsgSubTopic
                    dateTime = datetime(FullT{i,1},'ConvertFrom','posixtime');
                    dateTimeMS{i,1} = datestr(dateTime, 'dd-mmm-yyyy HH:MM:SS:FFF');
                end
                % Save the full File CSV into a sigle Table
                FullT.rosbagTimestamp = dateTimeMS;
                [namePath] = baseFileNames.folder;
                nameSplit = regexp(namePath,'/','split');
                nameCSVFile = strcat(nameSplit{end}, '.csv');
                writetable(FullT,nameCSVFile);
            end
		end
	else
		fprintf('Folder %s has not csv files in it.\n', thisFolder);
    end
end



