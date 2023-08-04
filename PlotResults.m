% Author     :   F. Moissenet
%                Biomechanics Laboratory (B-LAB)
%                Geneva University Hospital and University of Geneva
%                https://www.unige.ch/medecine/chiru/fr/b-lab-tests-robotises-avances-de-dispositifs-chirurgicaux/
% License    :   Creative Commons Attribution-NonCommercial 4.0 International License 
%                https://creativecommons.org/licenses/by-nc/4.0/legalcode
% Source code:   To be defined
% Reference  :   To be defined
% Date       :   July 2023
% -------------------------------------------------------------------------
% Description:   Plot kinematic results (for check)
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% INIT THE WORKSPACE
% -------------------------------------------------------------------------
clearvars;
close all;
warning off;
clc;

% -------------------------------------------------------------------------
% SET FOLDERS
% -------------------------------------------------------------------------
Folder.toolbox      = 'C:\Users\moissene\OneDrive - unige.ch\2019 - ROBOSHOULDER\Data\Distributed dataset and toolbox\BLAB_Roboshoulder_toolbox\';
Folder.dependencies = 'C:\Users\moissene\OneDrive - unige.ch\2019 - ROBOSHOULDER\Data\Distributed dataset and toolbox\BLAB_Roboshoulder_toolbox\dependencies\';
Folder.data         = 'C:\Users\moissene\OneDrive - unige.ch\2019 - ROBOSHOULDER\Data\Distributed dataset and toolbox\BLAB_Roboshoulder_dataset\';
addpath(Folder.toolbox);
addpath(genpath(Folder.dependencies));

subjectList   = {'RS001_L','RS001_R','RS002_L','RS002_R', ...
                 'RS003_L','RS003_R','RS004_L','RS004_R', ...
                 'RS005_L','RS005_R'};
conditionList = {'Native'};
motionList    = {'AA','FE','IER','HFE','VT','HC'};
jointList     = {'Humerothoracic','Glenohumeral','Scapulothoracic','Acromioclavicular','Sternoclavicular'};

% Create plots
for isubject = 1:size(subjectList,2)
    for icondition = 1
        for imotion = 1
            for ijoint = 0:size(jointList,2)-1
                cd([Folder.data,subjectList{isubject}]);
                if isfile([subjectList{isubject},'_',conditionList{icondition},'_',motionList{imotion},'_kinematics.csv'])
                    kinematics = readtable([subjectList{isubject},'_',conditionList{icondition},'_',motionList{imotion},'_kinematics.csv']);
                    figure(ijoint+1);
                    subplot(2,3,1); hold on; grid on; box on;
                    plot(table2array(kinematics(:,1)),table2array(kinematics(:,ijoint*6+1+1)));
                    subplot(2,3,2); hold on; grid on; box on;
                    plot(table2array(kinematics(:,1)),table2array(kinematics(:,ijoint*6+1+2)));
                    subplot(2,3,3); hold on; grid on; box on;
                    plot(table2array(kinematics(:,1)),table2array(kinematics(:,ijoint*6+1+3)));
                    subplot(2,3,4); hold on; grid on; box on;
                    plot(table2array(kinematics(:,1)),table2array(kinematics(:,ijoint*6+1+4)));
                    subplot(2,3,5); hold on; grid on; box on;
                    plot(table2array(kinematics(:,1)),table2array(kinematics(:,ijoint*6+1+5)));
                    subplot(2,3,6); hold on; grid on; box on;
                    plot(table2array(kinematics(:,1)),table2array(kinematics(:,ijoint*6+1+6)));
                    clear kinematics;
                end
            end
        end
    end
end