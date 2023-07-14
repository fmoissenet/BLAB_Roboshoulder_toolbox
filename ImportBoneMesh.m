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
% Description:   Import bone models from STL files
% -------------------------------------------------------------------------
% Dependencies : None
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

function Bone = ImportBoneMesh(Subject,Bone)

% -------------------------------------------------------------------------
% HUMERUS
% -------------------------------------------------------------------------
stlFile                      = [Subject.id,'_',Subject.side,'_Humerus.stl'];
temp                         = stlread(stlFile);
Bone(1).Static.Mesh.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(1).Static.Mesh.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;
% Light version (reduced number of vertices, for illustration purposes)
stlFile                       = [Subject.id,'_',Subject.side,'_Humerus_Light.stl'];
temp                          = stlread(stlFile);
Bone(1).Static.Meshl.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(1).Static.Meshl.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;

% -------------------------------------------------------------------------
% SCAPULA
% -------------------------------------------------------------------------
stlFile                      = [Subject.id,'_',Subject.side,'_Scapula.stl'];
temp                         = stlread(stlFile);
Bone(2).Static.Mesh.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(2).Static.Mesh.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;
% Light version (reduced number of vertices, for illustration purposes)
stlFile                       = [Subject.id,'_',Subject.side,'_Scapula_Light.stl'];
temp                          = stlread(stlFile);
Bone(2).Static.Meshl.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(2).Static.Meshl.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;

% -------------------------------------------------------------------------
% CLAVICLE
% -------------------------------------------------------------------------
stlFile                      = [Subject.id,'_',Subject.side,'_Clavicle.stl'];
temp                         = stlread(stlFile);
Bone(3).Static.Mesh.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(3).Static.Mesh.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;
% Light version (reduced number of vertices, for illustration purposes)
stlFile                       = [Subject.id,'_',Subject.side,'_Clavicle_Light.stl'];
temp                          = stlread(stlFile);
Bone(3).Static.Meshl.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(3).Static.Meshl.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;

% -------------------------------------------------------------------------
% THORAX
% -------------------------------------------------------------------------
stlFile                      = [Subject.id,'_',Subject.side,'_Thorax.stl'];
temp                         = stlread(stlFile);
Bone(4).Static.Mesh.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(4).Static.Mesh.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;
% Light version (reduced number of vertices, for illustration purposes)
stlFile                       = [Subject.id,'_',Subject.side,'_Thorax_Light.stl'];
temp                          = stlread(stlFile);
Bone(4).Static.Meshl.vertices = permute(temp.Points*1e-3,[2,1,3]); % m
Bone(4).Static.Meshl.faces    = permute(temp.ConnectivityList,[2,1,3]);
% Clear workspace
clear stlFile temp;