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

function Bone = SetGeometricObjects(Subject,Bone)

% -------------------------------------------------------------------------
% HUMERUS
% -------------------------------------------------------------------------
% Load point cloud
txtFile    = [Subject.id,'_',Subject.side,'_Humerus_Humeral_head.fcsv'];
temp       = readmatrix(txtFile,'Filetype','text');
pointCloud = temp(:,2:4)*1e-3; % m
nlandmark  = size(Bone(1).Static.Landmark,2)+1; % Add the centre of the humeral head sphere
for ipoint = 1:size(pointCloud,1)
    nlandmark = nlandmark+1;
    Bone(1).Static.Landmark(nlandmark).label       = ['Humerus articular surface contour',num2str(ipoint)];
    Bone(1).Static.Landmark(nlandmark).coordinates = pointCloud(ipoint,:)';
end
% Compute the least square plane, an orthonormal basis of the plane (V) and
% the normal (N)
P     = mean(pointCloud,1); % The mean of the samples belongs to the plane
R     = bsxfun(@minus,pointCloud,P); % Samples reduction
[V,D] = eig(R'*R); % Computation of the principal directions if the point cloud
N     = V(:,1); % Extract the normal from the eigenvectors
V     = V(:,2:end); % Extract the orthonormal basis from the eigenvectors
% Define the humerus articular surface coordinate system
X = V(:,1);
Y = V(:,2);
Z = N(:,1);
O = P';
% Express point cloud in the humerus articular surface coordinate system
pointCloud = inv([X Y Z])*(pointCloud'-O); 
% Set point cloud elevation based on the min elevation
pointCloud(3,:) = pointCloud(3,:)+abs(min(pointCloud(3,:)));
P               = mean(pointCloud,2);
% Apply the same transformations to all humerus mesh vertices
vertices      = inv([X Y Z])*(Bone(1).Static.Mesh.vertices-O);
vertices(3,:) = vertices(3,:)+abs(min(pointCloud(3,:)));
% Store the mesh related to the articular surface
subset   = find(vertices(3,:)>=0); % Find all mesh vertices higher than 0 along Z
vertices = vertices(:,subset);
subset   = find(vertices(2,:)>=-0.04);
vertices = vertices(:,subset);
subset   = find(vertices(2,:)<=0.04);
vertices = vertices(:,subset);
subset   = find(vertices(1,:)>=-0.04);
vertices = vertices(:,subset);
subset   = find(vertices(1,:)<=0.04);
vertices = vertices(:,subset);
% Reverse transformations to all humerus mesh vertices
vertices = [X Y Z]*vertices+O;
% Humerus head sphere
Bone(1).Static.Object(1).label          = 'Humeral head sphere';
[Bone(1).Static.Object(1).centre, ...
 Bone(1).Static.Object(1).radius]       = lssphere(vertices',mean(vertices')',0.025,0.001,0.001);
Bone(1).Static.Object(1).centre         = Bone(1).Static.Object(1).centre';
Bone(1).Static.Object(1).centre         = Bone(1).Static.Object(1).centre';
Bone(1).Static.Landmark(12).label       = 'Humeral head centre';
Bone(1).Static.Landmark(12).coordinates = Bone(1).Static.Object(1).centre;
clear txtFile temp pointCloud P R V D N X Y Z O vertices subset;

% -------------------------------------------------------------------------
% SCAPULA
% -------------------------------------------------------------------------
% Glenoid contact plane
txtFile    = [Subject.id,'_',Subject.side,'_Scapula_Glenoid.fcsv'];
temp       = readmatrix(txtFile,'Filetype','text');
pointCloud = temp(:,2:4)*1e-3; % m
nlandmark  = size(Bone(2).Static.Landmark,2);
for ipoint = 1:size(pointCloud,1)
    nlandmark = nlandmark+1;
    Bone(2).Static.Landmark(nlandmark).label       = ['Glenoid point ',num2str(ipoint)];
    Bone(2).Static.Landmark(nlandmark).coordinates = pointCloud(ipoint,:)';
end
contour = 1:size(pointCloud,1);
A       = [ones(size(pointCloud(contour,1))),pointCloud(contour,1),pointCloud(contour,2)]; % Solve X(3) = b(1) + b(2)*X(1) + b(3)*X(2): A = [1 X(1) X(2)]
b       = A\pointCloud(contour,3); % Solve X(3) = b(1) + b(2)*X(1) + b(3)*X(2): b = A\X(3), i.e. least square solution
b       = -b./sqrt(sum(b.^2,1));
Bone(2).Static.Object(1).label  = 'Glenoid contour normal';
Bone(2).Static.Object(1).normal = [b(2);b(3);b(1)];

% -------------------------------------------------------------------------
% CLAVICLE
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% THORAX
% -------------------------------------------------------------------------
