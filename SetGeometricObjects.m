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
Bone(1).Static.Object(1).centre         = Bone(1).Static.Object(1).centre;
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
% THORAX (adapted from https://doi.org/10.1007/s00276-019-02342-4)
% -------------------------------------------------------------------------
% Load mesh model related to scapula gliding surface (from rib 2 to 8)
stlFile     = [Subject.id,'_',Subject.side,'_Thorax_ScapulaGlidingSurface_Light.stl'];
[v,f,n]     = stlRead(stlFile);
FV.vertices = v*1e-3; % m
FV.faces    = f;
clear c v f;
% Compute vertex normals
FV.normals = patchnormals(FV); % Compute mean normal at each vertex
% Load landmarks defining the scapula gliding surface
txtFile = [Subject.id,'_',Subject.side,'_Thorax_ScapulaGlidingSurface_Landmarks.fcsv'];
temp    = csvread(txtFile,3,0);
points  = temp(1:6,2:4)*1e-3; % m
% Set plane coordinate system /left (landmarks 1 to 3)
X = (points(3,:)-points(1,:))/norm((points(3,:)-points(1,:)));
Y = (points(2,:)-points(1,:))/norm((points(2,:)-points(1,:)));
Z = cross(X,Y)/norm(cross(X,Y)); % Pointing posteriorly
X = cross(Y,Z)/norm(cross(Y,Z));
O = points(1,:);
% Keep only vertices of the positive side of the plane
PV.vertices = (FV.vertices-O)*inv([X;Y;Z]);
PV.normals = (FV.normals)*inv([X;Y;Z]);
subset = [];
for i = 1:size(FV.vertices,1)
    if PV.vertices(i,3) >= 0 && ... % Only posterior vertices
       PV.normals(i,3) <= 0 && ... % Only posterior oriented normals
       abs(PV.normals(i,3)) >= 0.25 % Normal orientation offset
        subset = [subset i];
    end
end
clear X Y Z O PV;
% Set plane coordinate system /right (landmarks 4 to 6)
X = -(points(6,:)-points(4,:))/norm((points(6,:)-points(4,:)));
Y = (points(5,:)-points(4,:))/norm((points(5,:)-points(4,:)));
Z = cross(X,Y)/norm(cross(X,Y)); % Pointing posteriorly
X = cross(Y,Z)/norm(cross(Y,Z));
O = points(4,:);
% Keep only vertices of the positive side of the plane
PV.vertices = (FV.vertices-O)*inv([X;Y;Z]);
PV.normals = (FV.normals)*inv([X;Y;Z]);
for i = 1:size(PV.vertices,1)
    if PV.vertices(i,3) >= 0 && ... % Only posterior vertices
       PV.normals(i,3) <= 0 && ... % Only posterior oriented normals
       abs(PV.normals(i,3)) >= 0.25 % Normal orientation offset
        subset = [subset i];
    end
end
clear X Y Z O PV;
% Store subset vertices and normals
SV.vertices = FV.vertices(subset,:);
SV.normals = FV.normals(subset,:);
% Scapula gliding surface (least-square ellipsoid)
[c,r,~,v]                               = ellipsoid_fit(SV.vertices);
x                                       = SV.vertices(:,1);
y                                       = SV.vertices(:,2);
z                                       = SV.vertices(:,3);
mind                                    = [min(x) min(y) min(z)];
maxd                                    = [max(x) max(y) max(z)];
nsteps                                  = 100;
step                                    = (maxd-mind)/nsteps;
[x y z]                                 = meshgrid(linspace(mind(1)-step(1),maxd(1)+step(1),nsteps), ...
                                               linspace(mind(2)-step(2),maxd(2)+step(2),nsteps), ...
                                               linspace(mind(3)-step(3),maxd(3)+step(3),nsteps));
Ellipsoid                               = v(1)*x.*x+v(2)*y.*y+v(3)*z.*z+ ...
                                          2*v(4)*x.*y+2*v(5)*x.*z+2*v(6)*y.*z + ...
                                          2*v(7)*x+2*v(8)*y+2*v(9)*z;
Bone(4).Static.Object(1).label          = 'Scapula gliding surface (ellipsoid)';
Bone(4).Static.Object(1).centre         = c;
Bone(4).Static.Object(1).radii          = r;
Bone(4).Static.Object(1).patch          = isosurface(x,y,z,Ellipsoid,-v(10));
Bone(4).Static.Object(1).patch.vertices = Bone(4).Static.Object(1).patch.vertices';
Bone(4).Static.Object(1).patch.faces    = Bone(4).Static.Object(1).patch.faces';
% Plot (ONLY FOR TEST)
% figure();
% hold on; axis equal; view(0,90);
% patch_array3(FV.faces',...
%              FV.vertices',...
%              [0.7 0.7 0.7],'none','gouraud',1);
% plot3(points(:,1),...
%       points(:,2),...
%       points(:,3),...
%       'Marker','.','MarkerSize',15,'Color','black');
% for i = 1:size(SV.vertices,1)
%     p1 = SV.vertices(i,:);
%     p2 = SV.vertices(i,:)-0.005*SV.normals(i,:);    
%     plot3(p1(1),p1(2),p1(3),'LineStyle','none','Marker','.','Color','red');   
%     plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'g-');
% end
% patch_array3(Bone(4).Static.Object(1).patch.faces,...
%              Bone(4).Static.Object(1).patch.vertices,...
%              'blue','none','gouraud',0.3);
% test;