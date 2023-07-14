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
% Description:   Import cluster markers from FCSV files
% -------------------------------------------------------------------------
% Dependencies : - LSGE: The Least Squares Geometric Elements library: http://www.eurometros.org/gen_report.php?category=distributions&pkey=14
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

function Bone = ImportClusterMarkers(Subject,Bone)

% -------------------------------------------------------------------------
% HUMERUS
% -------------------------------------------------------------------------
for imarker = 1:4
    txtFile                                      = [Subject.id,'_',Subject.side,'_Humerus_c',Subject.side,'HUM0',num2str(imarker),'.fcsv'];
    temp                                         = readmatrix(txtFile,'Filetype','text');
    pointCloud                                   = temp(:,2:4)*1e-3; % m
    Bone(1).Static.Landmark(imarker).label       = ['c',Subject.side,'HUM0',num2str(imarker)];
    Bone(1).Static.Landmark(imarker).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
    clear txtFile temp pointCloud; 
end

% -------------------------------------------------------------------------
% SCAPULA
% -------------------------------------------------------------------------
for imarker = 1:4
    txtFile                                      = [Subject.id,'_',Subject.side,'_Scapula_c',Subject.side,'SCA0',num2str(imarker),'.fcsv'];
    temp                                         = readmatrix(txtFile,'Filetype','text');
    pointCloud                                   = temp(:,2:4)*1e-3; % m
    Bone(2).Static.Landmark(imarker).label       = ['c',Subject.side,'SCA0',num2str(imarker)];
    Bone(2).Static.Landmark(imarker).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
    clear txtFile temp pointCloud; 
end

% -------------------------------------------------------------------------
% CLAVICLE
% -------------------------------------------------------------------------
for imarker = 1:4
    txtFile                                      = [Subject.id,'_',Subject.side,'_Clavicle_c',Subject.side,'CLA0',num2str(imarker),'.fcsv'];
    temp                                         = readmatrix(txtFile,'Filetype','text');
    pointCloud                                   = temp(:,2:4)*1e-3; % m
    Bone(3).Static.Landmark(imarker).label       = ['c',Subject.side,'CLA0',num2str(imarker)];
    Bone(3).Static.Landmark(imarker).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
    clear txtFile temp pointCloud; 
end

% -------------------------------------------------------------------------
% THORAX
% -------------------------------------------------------------------------
for imarker = 1:4
    txtFile                                      = [Subject.id,'_',Subject.side,'_Thorax_c','THO0',num2str(imarker),'.fcsv'];
    temp                                         = readmatrix(txtFile,'Filetype','text');
    pointCloud                                   = temp(2:end,2:4)*1e-3; % m
    Bone(4).Static.Landmark(imarker).label       = ['cTHO0',num2str(imarker)];
    Bone(4).Static.Landmark(imarker).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
    clear txtFile temp pointCloud; 
end