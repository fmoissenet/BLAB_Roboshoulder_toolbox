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
% Description:   Main routine to process kinematic parameters
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
% SET SUBJECT INFO
% -------------------------------------------------------------------------
Subject.id   = 'RS001';
Subject.side = 'L';

% -------------------------------------------------------------------------
% SET FOLDERS
% -------------------------------------------------------------------------
Folder.toolbox      = 'C:\Users\moissene\OneDrive - unige.ch\2019 - ROBOSHOULDER\Data\Roboshoulder_toolbox\';
Folder.dependencies = 'C:\Users\moissene\OneDrive - unige.ch\2019 - ROBOSHOULDER\Data\Roboshoulder_toolbox\dependencies\';
Folder.data         = ['C:\Users\moissene\OneDrive - unige.ch\2019 - ROBOSHOULDER\Data\Roboshoulder_dataset\',Subject.id,'_',Subject.side,'\'];
Folder.export       = [Folder.data,'\Results\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dependencies));

% -------------------------------------------------------------------------
% LOAD BONE MODELS
% -------------------------------------------------------------------------
% Reach imaging data folder
cd([Folder.data,'\Imaging']);
% Initialise bones
Bone(1).label = 'Humerus';
Bone(2).label = 'Scapula';
Bone(3).label = 'Clavicle';
Bone(4).label = 'Thorax';
% Import bone meshes
Bone = ImportBoneMesh(Subject,Bone);
% Import cluster markers
Bone = ImportClusterMarkers(Subject,Bone);
% Import bony landmarks
Bone = ImportBonyLandmarks(Subject,Bone);
% Set geometric objects
Bone = SetGeometricObjects(Subject,Bone);

% -------------------------------------------------------------------------
% APPLY STATIC POSE ON BONE MODELS
% -------------------------------------------------------------------------
% Initialisation
step         = 1; % Frame step
showVertices = 1; % Compute/show vertices (1) or not (0)
% Reach imaging data folder
cd([Folder.data,'\Motion']);
% Load static pose from native condition
c3dFile = [Subject.id,'_',Subject.side,'_N.c3d'];
Static  = LoadMotionFile(Subject,[],[],c3dFile,'Static','Static',1);
clear c3dFile;
% Set segment coordinate systems
Bone = SetSegmentCoordinateSystems(Bone,'Static',1);
% Update whole posture based on static pose
Bone = UpdateBonePose(Bone,Static,'Static',1,step,showVertices);
% Plot
PlotStatic(Bone,showVertices);

% -------------------------------------------------------------------------
% APPLY MOTION FROM RECORDED DYNAMIC TASK
% -------------------------------------------------------------------------
% Initialisation
step         = 1; % Frame step
showVertices = 1; % Compute/show vertices (1) or not (0)
fig          = 0; % Plot kinematic results (1) or not (0)
file         = 0; % Generate kinematics result file (1) or not (0)
close all;
% Reach imaging data folder
cd([Folder.data,'\Motion']);
% Set trial, condition, motion lists
trialList     = {[Subject.id,'_',Subject.side,'_N']};
conditionList = {'Native'}; % Must have the same size than trialList
motionList    = {'AA','FE','IER','HFE','VT','HC'};
for itrial = 1:size(trialList,2)
    for imotion = 1:size(motionList,2)
        % Initialise Bone structure
        for ibone = 1:4
            Bone(ibone).Trial = [];
        end
        % Load motion
        c3dFile = [trialList{itrial},'.c3d'];
        if itrial == 1 % Initialise the Trial structure
            Trial   = LoadMotionFile(Subject,Static,[],c3dFile,conditionList{itrial},motionList{imotion},itrial);
        else
            Trial   = LoadMotionFile(Subject,Static,Trial,c3dFile,conditionList{itrial},motionList{imotion},itrial);
        end
        clear c3dFile;
        % Compute landmarks trajectories
        Bone = UpdateBonePose(Bone,Trial(itrial),'Trial',itrial,step,showVertices);
        % Set segment coordinate systems
        Bone = SetSegmentCoordinateSystems(Bone,'Trial',itrial);
        % Compute kinematics
        Trial(itrial).Joint = ComputeKinematics(Subject,Bone,Trial,itrial,fig,file);
        % Plot
        PlotDynamic(Bone,Trial,'Trial',itrial,'back',showVertices);
    end
end