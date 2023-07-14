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
% Description:   Import bony landmarks from FCSV files
% -------------------------------------------------------------------------
% Dependencies : - LSGE: The Least Squares Geometric Elements library: http://www.eurometros.org/gen_report.php?category=distributions&pkey=14
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

function Bone = ImportBonyLandmarks(Subject,Bone)

% -------------------------------------------------------------------------
% HUMERUS
% -------------------------------------------------------------------------
txtFile                                 = [Subject.id,'_',Subject.side,'_Humerus_Landmarks.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(:,2:4)*1e-3; % m
Bone(1).Static.Landmark(5).label        = [Subject.side,'HLE'];
Bone(1).Static.Landmark(5).coordinates  = pointCloud(7,:)';
Bone(1).Static.Landmark(6).label        = [Subject.side,'HME'];
Bone(1).Static.Landmark(6).coordinates  = pointCloud(5,:)';
Bone(1).Static.Landmark(7).label        = [Subject.side,'HMU'];
Bone(1).Static.Landmark(7).coordinates  = pointCloud(4,:)';
Bone(1).Static.Landmark(8).label        = [Subject.side,'HML'];
Bone(1).Static.Landmark(8).coordinates  = pointCloud(6,:)';
Bone(1).Static.Landmark(9).label        = [Subject.side,'HGT'];
Bone(1).Static.Landmark(9).coordinates  = pointCloud(1,:)';
Bone(1).Static.Landmark(10).label       = [Subject.side,'HLT'];
Bone(1).Static.Landmark(10).coordinates = pointCloud(2,:)';
Bone(1).Static.Landmark(11).label       = [Subject.side,'HDT'];
Bone(1).Static.Landmark(11).coordinates = pointCloud(3,:)';
clear txtFile temp pointCloud;

% -------------------------------------------------------------------------
% SCAPULA
% -------------------------------------------------------------------------
txtFile                                 = [Subject.id,'_',Subject.side,'_Scapula_Landmarks.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(:,2:4)*1e-3; % m
Bone(2).Static.Landmark(5).label        = [Subject.side,'SIA'];
Bone(2).Static.Landmark(5).coordinates  = pointCloud(1,:)';
Bone(2).Static.Landmark(6).label        = [Subject.side,'SSA'];
Bone(2).Static.Landmark(6).coordinates  = pointCloud(3,:)';
Bone(2).Static.Landmark(7).label        = [Subject.side,'SAA'];
Bone(2).Static.Landmark(7).coordinates  = pointCloud(4,:)';
Bone(2).Static.Landmark(8).label        = [Subject.side,'SAT'];
Bone(2).Static.Landmark(8).coordinates  = pointCloud(5,:)';
Bone(2).Static.Landmark(9).label        = [Subject.side,'SAE'];
Bone(2).Static.Landmark(9).coordinates  = pointCloud(6,:)';
Bone(2).Static.Landmark(10).label       = [Subject.side,'SCT'];
Bone(2).Static.Landmark(10).coordinates = pointCloud(7,:)';
Bone(2).Static.Landmark(11).label       = [Subject.side,'SAJ'];
Bone(2).Static.Landmark(11).coordinates = pointCloud(8,:)';
Bone(2).Static.Landmark(12).label       = [Subject.side,'SRS'];
Bone(2).Static.Landmark(12).coordinates = pointCloud(2,:)';
clear txtFile temp pointCloud;

% -------------------------------------------------------------------------
% CLAVICLE
% -------------------------------------------------------------------------
txtFile                                 = [Subject.id,'_',Subject.side,'_Clavicle_Landmarks.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(:,2:4)*1e-3; % m
Bone(3).Static.Landmark(5).label        = [Subject.side,'CAE'];
Bone(3).Static.Landmark(5).coordinates  = pointCloud(3,:)';
Bone(3).Static.Landmark(6).label        = [Subject.side,'CAA'];
Bone(3).Static.Landmark(6).coordinates  = pointCloud(2,:)';
Bone(3).Static.Landmark(7).label        = [Subject.side,'CAS'];
Bone(3).Static.Landmark(7).coordinates  = pointCloud(5,:)';
Bone(3).Static.Landmark(8).label        = [Subject.side,'CSJ'];
Bone(3).Static.Landmark(8).coordinates  = pointCloud(4,:)';
Bone(3).Static.Landmark(9).label        = [Subject.side,'CAJ'];
Bone(3).Static.Landmark(9).coordinates  = pointCloud(1,:)';
clear txtFile temp pointCloud;

% -------------------------------------------------------------------------
% THORAX
% -------------------------------------------------------------------------
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_Landmarks.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(5).label        = 'SJN';
Bone(4).Static.Landmark(5).coordinates  = pointCloud(1,:)';
Bone(4).Static.Landmark(6).label        = 'RSCS';
Bone(4).Static.Landmark(6).coordinates  = pointCloud(2,:)';
Bone(4).Static.Landmark(7).label        = 'LSCS';
Bone(4).Static.Landmark(7).coordinates  = pointCloud(3,:)';
Bone(4).Static.Landmark(8).label        = 'SME';
Bone(4).Static.Landmark(8).coordinates  = pointCloud(4,:)';
% For thorax, some landmarks have been pointed on CT scan 2D views to
% create virtual markers, to allow the computation of their centroid to
% determine the 3D position of the landmark (see RS00i_i_Thorax_Suppl.stl
% for geometry)
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_CJC.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(9).label        = 'CJC';
Bone(4).Static.Landmark(9).coordinates  = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
clear txtFile temp pointCloud;
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_TJC.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(10).label       = 'TJC';
Bone(4).Static.Landmark(10).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
clear txtFile temp pointCloud;
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_CV7.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(11).label       = 'CV7';
Bone(4).Static.Landmark(11).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
clear txtFile temp pointCloud;
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_TV8.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(12).label       = 'TV8';
Bone(4).Static.Landmark(12).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
clear txtFile temp pointCloud;
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_TV12.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(13).label       = 'TV12';
Bone(4).Static.Landmark(13).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
clear txtFile temp pointCloud;
txtFile                                 = [Subject.id,'_',Subject.side,'_Thorax_SXS.fcsv'];
temp                                    = readmatrix(txtFile,'Filetype','text');
pointCloud                              = temp(2:end,2:4)*1e-3; % m
Bone(4).Static.Landmark(14).label       = 'SXS';
Bone(4).Static.Landmark(14).coordinates = lssphere(pointCloud,mean(pointCloud)',0.01,0.001,0.001);
clear txtFile temp pointCloud;