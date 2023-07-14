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
% Description:   Plot static bone pose
% -------------------------------------------------------------------------
% Dependencies : - patch_array3
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

function [] = PlotStatic(Bone,showVertices)

figure(1);
quiver3(0,0,0,1,0,0,0.02,'Color','red','LineWidth',2);
hold on; axis equal; view(0,90);
quiver3(0,0,0,0,1,0,0.02,'Color','green','LineWidth',2);
quiver3(0,0,0,0,0,1,0.02,'Color','blue','LineWidth',2);

% -------------------------------------------------------------------------
% HUMERUS
% -------------------------------------------------------------------------
if showVertices == 1
    patch_array3(Bone(1).Static.Meshl.faces,...
                 Bone(1).Static.Meshl.vertices_t(:,:,1),...
                 [0.7 0.7 0.7],'none','gouraud',0.2);
end
xlabel('x'); ylabel('y'); zlabel('z');
for imarker = 1:4
    plot3(Bone(1).Static.Landmark(imarker).coordinates_t(1,1,1),...
          Bone(1).Static.Landmark(imarker).coordinates_t(2,1,1),...
          Bone(1).Static.Landmark(imarker).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','red');
end
for ilandmark = 5:12
    plot3(Bone(1).Static.Landmark(ilandmark).coordinates_t(1,1,1),...
          Bone(1).Static.Landmark(ilandmark).coordinates_t(2,1,1),...
          Bone(1).Static.Landmark(ilandmark).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','green');
end
for ilandmark = 12+1:size(Bone(1).Static.Landmark,2)
    plot3(Bone(1).Static.Landmark(ilandmark).coordinates_t(1,1,1),...
          Bone(1).Static.Landmark(ilandmark).coordinates_t(2,1,1),...
          Bone(1).Static.Landmark(ilandmark).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','blue');
end
[x,y,z] = sphere;
surf(x*Bone(1).Static.Object(1).radius+Bone(1).Static.Object(1).centre_t(1,1,1), ...
     y*Bone(1).Static.Object(1).radius+Bone(1).Static.Object(1).centre_t(2,1,1), ...
     z*Bone(1).Static.Object(1).radius+Bone(1).Static.Object(1).centre_t(3,1,1),...
     'Facecolor','blue','FaceAlpha',0.3);
quiver3(Bone(1).Static.SCS.O_t(1,1,1),Bone(1).Static.SCS.O_t(2,1,1),Bone(1).Static.SCS.O_t(3,1,1), ...
        Bone(1).Static.SCS.X_t(1,1,1),Bone(1).Static.SCS.X_t(2,1,1),Bone(1).Static.SCS.X_t(3,1,1),...
        0.1,'Color','red');
quiver3(Bone(1).Static.SCS.O_t(1,1,1),Bone(1).Static.SCS.O_t(2,1,1),Bone(1).Static.SCS.O_t(3,1,1), ...
        Bone(1).Static.SCS.Y_t(1,1,1),Bone(1).Static.SCS.Y_t(2,1,1),Bone(1).Static.SCS.Y_t(3,1,1),...
        0.1,'Color','green');
quiver3(Bone(1).Static.SCS.O_t(1,1,1),Bone(1).Static.SCS.O_t(2,1,1),Bone(1).Static.SCS.O_t(3,1,1), ...
        Bone(1).Static.SCS.Z_t(1,1,1),Bone(1).Static.SCS.Z_t(2,1,1),Bone(1).Static.SCS.Z_t(3,1,1),...
        0.1,'Color','blue'); 
    
% -------------------------------------------------------------------------
% SCAPULA
% -------------------------------------------------------------------------    
if showVertices == 1
    patch_array3(Bone(2).Static.Meshl.faces,...
                 Bone(2).Static.Meshl.vertices_t(:,:,1),...
                 [0.7 0.7 0.7],'none','gouraud',0.2);
end
for imarker = 1:4
    plot3(Bone(2).Static.Landmark(imarker).coordinates_t(1,1,1),...
          Bone(2).Static.Landmark(imarker).coordinates_t(2,1,1),...
          Bone(2).Static.Landmark(imarker).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','red');
end
for ilandmark = 5:12
    plot3(Bone(2).Static.Landmark(ilandmark).coordinates_t(1,1,1),...
          Bone(2).Static.Landmark(ilandmark).coordinates_t(2,1,1),...
          Bone(2).Static.Landmark(ilandmark).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','green');
end
for ilandmark = 12+1:size(Bone(2).Static.Landmark,2)
    plot3(Bone(2).Static.Landmark(ilandmark).coordinates_t(1,1,1),...
          Bone(2).Static.Landmark(ilandmark).coordinates_t(2,1,1),...
          Bone(2).Static.Landmark(ilandmark).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','blue');
end
quiver3(Bone(2).Static.SCS.O_t(1,1,1),Bone(2).Static.SCS.O_t(2,1,1),Bone(2).Static.SCS.O_t(3,1,1), ...
        Bone(2).Static.SCS.X_t(1,1,1),Bone(2).Static.SCS.X_t(2,1,1),Bone(2).Static.SCS.X_t(3,1,1),...
        0.1,'Color','red');
quiver3(Bone(2).Static.SCS.O_t(1,1,1),Bone(2).Static.SCS.O_t(2,1,1),Bone(2).Static.SCS.O_t(3,1,1), ...
        Bone(2).Static.SCS.Y_t(1,1,1),Bone(2).Static.SCS.Y_t(2,1,1),Bone(2).Static.SCS.Y_t(3,1,1),...
        0.1,'Color','green');
quiver3(Bone(2).Static.SCS.O_t(1,1,1),Bone(2).Static.SCS.O_t(2,1,1),Bone(2).Static.SCS.O_t(3,1,1), ...
        Bone(2).Static.SCS.Z_t(1,1,1),Bone(2).Static.SCS.Z_t(2,1,1),Bone(2).Static.SCS.Z_t(3,1,1),...
        0.1,'Color','blue'); 

% -------------------------------------------------------------------------
% CLAVICLE
% -------------------------------------------------------------------------
if showVertices == 1
    patch_array3(Bone(3).Static.Meshl.faces,...
                 Bone(3).Static.Meshl.vertices_t(:,:,1),...
                 [0.7 0.7 0.7],'none','gouraud',0.2);
end
for imarker = 1:4
    plot3(Bone(3).Static.Landmark(imarker).coordinates_t(1,1,1),...
          Bone(3).Static.Landmark(imarker).coordinates_t(2,1,1),...
          Bone(3).Static.Landmark(imarker).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','red');
end
for ilandmark = 5:9
    plot3(Bone(3).Static.Landmark(ilandmark).coordinates_t(1,1,1),...
          Bone(3).Static.Landmark(ilandmark).coordinates_t(2,1,1),...
          Bone(3).Static.Landmark(ilandmark).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','green');
end
quiver3(Bone(3).Static.SCS.O_t(1,1,1),Bone(3).Static.SCS.O_t(2,1,1),Bone(3).Static.SCS.O_t(3,1,1), ...
        Bone(3).Static.SCS.X_t(1,1,1),Bone(3).Static.SCS.X_t(2,1,1),Bone(3).Static.SCS.X_t(3,1,1),...
        0.1,'Color','red');
quiver3(Bone(3).Static.SCS.O_t(1,1,1),Bone(3).Static.SCS.O_t(2,1,1),Bone(3).Static.SCS.O_t(3,1,1), ...
        Bone(3).Static.SCS.Y_t(1,1,1),Bone(3).Static.SCS.Y_t(2,1,1),Bone(3).Static.SCS.Y_t(3,1,1),...
        0.1,'Color','green');
quiver3(Bone(3).Static.SCS.O_t(1,1,1),Bone(3).Static.SCS.O_t(2,1,1),Bone(3).Static.SCS.O_t(3,1,1), ...
        Bone(3).Static.SCS.Z_t(1,1,1),Bone(3).Static.SCS.Z_t(2,1,1),Bone(3).Static.SCS.Z_t(3,1,1),...
        0.1,'Color','blue');

% -------------------------------------------------------------------------
% THORAX
% -------------------------------------------------------------------------
if showVertices == 1
    patch_array3(Bone(4).Static.Meshl.faces,...
                 Bone(4).Static.Meshl.vertices_t(:,:,1),...
                 [0.7 0.7 0.7],'none','gouraud',0.2);
end
for imarker = 1:4
    plot3(Bone(4).Static.Landmark(imarker).coordinates_t(1,1,1),...
          Bone(4).Static.Landmark(imarker).coordinates_t(2,1,1),...
          Bone(4).Static.Landmark(imarker).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','red');
end
for ilandmark = 5:14
    plot3(Bone(4).Static.Landmark(ilandmark).coordinates_t(1,1,1),...
          Bone(4).Static.Landmark(ilandmark).coordinates_t(2,1,1),...
          Bone(4).Static.Landmark(ilandmark).coordinates_t(3,1,1),...
          'Marker','.','MarkerSize',15,'Color','green');
end
quiver3(Bone(4).Static.SCS.O_t(1,1,1),Bone(4).Static.SCS.O_t(2,1,1),Bone(4).Static.SCS.O_t(3,1,1), ...
        Bone(4).Static.SCS.X_t(1,1,1),Bone(4).Static.SCS.X_t(2,1,1),Bone(4).Static.SCS.X_t(3,1,1),...
        0.1,'Color','red');
quiver3(Bone(4).Static.SCS.O_t(1,1,1),Bone(4).Static.SCS.O_t(2,1,1),Bone(4).Static.SCS.O_t(3,1,1), ...
        Bone(4).Static.SCS.Y_t(1,1,1),Bone(4).Static.SCS.Y_t(2,1,1),Bone(4).Static.SCS.Y_t(3,1,1),...
        0.1,'Color','green');
quiver3(Bone(4).Static.SCS.O_t(1,1,1),Bone(4).Static.SCS.O_t(2,1,1),Bone(4).Static.SCS.O_t(3,1,1), ...
        Bone(4).Static.SCS.Z_t(1,1,1),Bone(4).Static.SCS.Z_t(2,1,1),Bone(4).Static.SCS.Z_t(3,1,1),...
        0.1,'Color','blue');