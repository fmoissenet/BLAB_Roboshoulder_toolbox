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
% Description:   Import motion from C3D file
% -------------------------------------------------------------------------
% Dependencies : - 3D Kinematics and Inverse Dynamics toolbox by Raphaël Dumas: https://fr.mathworks.com/matlabcentral/fileexchange/58021-3d-kinematics-and-inverse-dynamics
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

function Joint = ComputeKinematics(Subject,Bone,Trial,itrial,fig,file)

n = length(1:size(Bone(1).Trial(itrial).SCS.T,3));

% -------------------------------------------------------------------------
% HUMEROTHORACIC JOINT
% -------------------------------------------------------------------------
% Initialisation
Joint(1).label        = 'Humerothoracic joint';
% Homogenous matrix of the rigid transformation between segments
Joint(1).T            = Mprod_array3(Tinv_array3(Bone(4).Trial(itrial).SCS.T),...
                                     Bone(1).Trial(itrial).SCS.T);
% JCS and motion for the humerus relative to the thorax (Y-X-Y order) 
% (Wu et al. 2005)
Joint(1).sequence     = 'YXY';
Joint(1).legend       = {'Elevation plane','Elevation angle','Internal (+) / External (-) rotation',...
                         'Anterior (+) / Posterior (-) translation','Inferior (-) / Superior (+) translation','Medial (-) / Lateral (+) translation'};
Euler                 = R2mobileYXY_array3(Joint(1).T(1:3,1:3,:));
Joint(1).Euler(1,1,:) = rad2deg(Euler(:,1,:)); % Elevation plane
Joint(1).Euler(1,2,:) = rad2deg(Euler(:,2,:)); % Elevation angle
Joint(1).Euler(1,3,:) = rad2deg(Euler(:,3,:)); % Internal-external rotation 
Joint(1).dj           = [];    
% Special case: displacements expressed in thorax coordinate system
dj                    = Joint(1).T(1:3,4,:); % Di+1 to Pi in SCS of segment i+1
Joint(1).dj           = dj*1e3; % Stored in mm
% Left side corrections
if contains(Subject.side,'L')
    Joint(1).Euler(1,1,:) = -Joint(1).Euler(1,1,:);
    Joint(1).Euler(1,3,:) = -Joint(1).Euler(1,3,:)+180;
    Joint(1).dj(3,1,:)    = -Joint(1).dj(3,1,:);
end
clear Euler dj x y p x1 y1;

% -------------------------------------------------------------------------
% GLENOHUMERAL JOINT
% -------------------------------------------------------------------------
% Initialisation
Joint(2).label        = 'Glenohumeral joint';
% Homogenous matrix of the rigid transformation between segments
Joint(2).T            = Mprod_array3(Tinv_array3(Bone(2).Trial(itrial).SCS.T),...
                                     Bone(1).Trial(itrial).SCS.T);                         
% JCS and motion for the humerus relative to the scapula (X–Z–Y order) 
% (Senk and Chèze 2006, Phadke et al. 2011)
Joint(2).sequence     = 'XZY';
Joint(2).legend       = {'Abduction (-) / Adduction (+)','Flexion (+) / Extension (-)','Internal (+) / External (-) rotation',...
                         'Anterior (+) / Posterior (-) translation','Medial (-) / Lateral (+) translation','Inferior (-) / Superior (+) translation'};
Euler                 = R2mobileXZY_array3(Joint(2).T(1:3,1:3,:));
Joint(2).Euler(1,1,:) = rad2deg(Euler(:,1,:)); % Abduction-adduction
Joint(2).Euler(1,2,:) = rad2deg(Euler(:,2,:)); % Flexion-extension
Joint(2).Euler(1,3,:) = rad2deg(Euler(:,3,:)); % Internal-external rotation
dj                    = Vnop_array3(Joint(2).T(1:3,4,:),... Di+1 to Pi in SCS of segment i+1
                                    repmat([1;0;0],[1 1 n]),... % Xi+1 in SCS of segment i+1
                                    Vnorm_array3(cross(repmat([1;0;0],[1 1 n]),Joint(2).T(1:3,2,:))),... Z = XxY
                                    Joint(2).T(1:3,2,:)); % Yi in SCS of segment i  
Joint(2).dj           = dj*1e3; % Stored in mm
% Left side corrections
if contains(Subject.side,'L')
    Joint(2).Euler(1,2,:) = -Joint(2).Euler(1,2,:);
    Joint(2).Euler(1,3,:) = -Joint(2).Euler(1,3,:);
    Joint(2).dj(1,1,:)    = -Joint(2).dj(1,1,:);
end
clear Euler dj;

% -------------------------------------------------------------------------
% SCAPULOTHORACIC JOINT
% -------------------------------------------------------------------------
% Initialisation
Joint(3).label        = 'Scapulothoracic joint';
% Homogenous matrix of the rigid transformation between segments
Joint(3).T            = Mprod_array3(Tinv_array3(Bone(4).Trial(itrial).SCS.T),...
                                     Bone(2).Trial(itrial).SCS.T);  
% JCS and motion for the scapula relative to the thorax (Y–X–Z order) 
% (Wu et al. 2005)
Joint(3).sequence     = 'YXZ';
Joint(3).legend       = {'Retraction (-) / Protraction (+)','Lateral (-) / Medial (+) rotation','Anterior (-) / Posterior (+) tilt',...
                         'Inferior (-) / Superior (+) translation','Anterior (+) / Posterior (-) translation','Medial (-) / Lateral (+) translation'};
Euler                 = R2mobileYXZ_array3(Joint(3).T(1:3,1:3,:));
Joint(3).Euler(1,1,:) = rad2deg(Euler(:,1,:)); % Internal-External Rotation or Protraction-Retraction
Joint(3).Euler(1,2,:) = rad2deg(Euler(:,2,:)); % Medial-Lateral or Downward-Upward Rotation for scapula
Joint(3).Euler(1,3,:) = rad2deg(Euler(:,3,:)); % Flexion-Extension or Posterior-Anterior Tilt
dj                    = Vnop_array3(Joint(3).T(1:3,4,:),... Di+1 to Pi in SCS of segment i+1
                                    repmat([0;1;0],[1 1 n]),... % Yi+1 in SCS of segment i+1
                                    Vnorm_array3(cross(repmat([0;1;0],[1 1 n]),Joint(3).T(1:3,3,:))),... X = YxZ
                                    Joint(3).T(1:3,3,:)); % Zi in SCS of segment i 
Joint(3).dj           = dj*1e3; % Stored in mm
% Left side corrections
if contains(Subject.side,'L')
    Joint(3).Euler(1,1,:) = -Joint(3).Euler(1,1,:)+180;
    Joint(3).dj(2,1,:)    = -Joint(3).dj(2,1,:);
end
clear Euler dj;

% -------------------------------------------------------------------------
% ACROMIOCLAVICULAR JOINT
% -------------------------------------------------------------------------
% Initialisation
Joint(4).label        = 'Acromioclavicular joint';
% Homogenous matrix of the rigid transformation between segments
Joint(4).T            = Mprod_array3(Tinv_array3(Bone(3).Trial(itrial).SCS.T),...
                                     Bone(2).Trial(itrial).SCS.T);  
% JCS and motion for the scapula relative to the clavicle (Y–X–Z order) 
% (Wu et al. 2005)
Joint(4).sequence     = 'YXZ';
Joint(4).legend       = {'Retraction (-) / Protraction (+)','Lateral (-) / Medial (+) rotation','Anterior (-) / Posterior (+) tilt',...
                         'Inferior (-) / Superior (+) translation','Anterior (+) / Posterior (-) translation','Medial (-) / Lateral (+) translation'};
Euler                 = R2mobileYXZ_array3(Joint(4).T(1:3,1:3,:));
Joint(4).Euler(1,1,:) = rad2deg(Euler(:,1,:)); % Internal-External Rotation or Protraction-Retraction
Joint(4).Euler(1,2,:) = rad2deg(Euler(:,2,:)); % Medial-Lateral or Downward-Upward Rotation for scapula
Joint(4).Euler(1,3,:) = rad2deg(Euler(:,3,:)); % Flexion-Extension or Posterior-Anterior Tilt
% Translation of the origin of the scapula related to the origin of
% the clavicle
dj                    = Vnop_array3(Joint(4).T(1:3,4,:),... Di+1 to Pi in SCS of segment i+1
                                    repmat([0;1;0],[1 1 n]),... % Yi+1 in SCS of segment i+1
                                    Vnorm_array3(cross(repmat([0;1;0],[1 1 n]),Joint(4).T(1:3,3,:))),... X = YxZ
                                    Joint(4).T(1:3,3,:)); % Zi in SCS of segment i                                     
Joint(4).dj           = dj*1e3; % Stored in mm
%     % Example: displacement of the acroclavicular joint point expressed in
%     % the scapula frame vs. expressed in the clavicle frame
%     dj                    = Vnop_array3(Bone(2).Trial(itrial).Vlandmark(1).coordinate(:,:,:)-Bone(3).Trial(itrial).Landmark(9).coordinates(:,:,:),...% Joint(4).T(1:3,4,:),... Di+1 to Pi in SCS of segment i+1
%                                         Bone(3).Trial(itrial).SCS.Y,... %repmat([0;1;0],[1 1 n]),... % Yi+1 in SCS of segment i+1
%                                         Vnorm_array3(cross(Bone(3).Trial(itrial).SCS.Y,Bone(2).Trial(itrial).SCS.Z)),... %Vnorm_array3(cross(repmat([0;1;0],[1 1 n]),Joint(4).T(1:3,3,:))),... X = YxZ
%                                         Bone(2).Trial(itrial).SCS.Z); %Joint(4).T(1:3,3,:)); % Zi in SCS of segment i                                     
%     Joint(4).dj           = dj*1e3; % Stored in mm
%     clear Euler dj;  
% Left side corrections
if contains(Subject.side,'L')
    Joint(4).Euler(1,1,:) = -Joint(4).Euler(1,1,:);
    Joint(4).Euler(1,3,:) = -Joint(4).Euler(1,3,:);
    Joint(4).dj(2,1,:)    = -Joint(4).dj(2,1,:);
end

% -------------------------------------------------------------------------
% STERNOCLAVICULAR JOINT
% -------------------------------------------------------------------------
% Initialisation
Joint(5).label        = 'Sternoclavicular joint';
% Homogenous matrix of the rigid transformation between segments
Joint(5).T            = Mprod_array3(Tinv_array3(Bone(4).Trial(itrial).SCS.T),...
                                     Bone(3).Trial(itrial).SCS.T);  
% JCS and motion for the clavicle relative to the thorax (Y–X–Z order) 
% (Wu et al. 2005)
Joint(5).sequence     = 'YXZ';
Joint(5).legend       = {'Retraction (-) / Protraction (+)','Elevation (-) / Depression (+)','Backward (+) / Forward (-) axial rotation',...
                         'Inferior (-) / Superior (+) translation','Anterior (+) / Posterior (-) translation','Medial (-) / Lateral (+) translation'};
Euler                 = R2mobileYXZ_array3(Joint(5).T(1:3,1:3,:));
Joint(5).Euler(1,1,:) = rad2deg(Euler(:,1,:)); % Internal-External Rotation or Protraction-Retraction
Joint(5).Euler(1,2,:) = rad2deg(Euler(:,2,:)); % Medial-Lateral or Downward-Upward Rotation for scapula
Joint(5).Euler(1,3,:) = rad2deg(Euler(:,3,:)); % Flexion-Extension or Posterior-Anterior Tilt
dj                    = Vnop_array3(Joint(5).T(1:3,4,:),... Di+1 to Pi in SCS of segment i+1
                                    repmat([0;1;0],[1 1 n]),... % Yi+1 in SCS of segment i+1
                                    Vnorm_array3(cross(repmat([0;1;0],[1 1 n]),Joint(5).T(1:3,3,:))),... X = YxZ
                                    Joint(5).T(1:3,3,:)); % Zi in SCS of segment i 
Joint(5).dj           = dj*1e3; % Stored in mm
clear Euler dj;
% Left side corrections
if contains(Subject.side,'L')
    Joint(5).Euler(1,1,:) = -Joint(5).Euler(1,1,:)-180;
    Joint(5).Euler(1,2,:) = -Joint(5).Euler(1,2,:);
    Joint(5).dj(2,1,:)    = -Joint(5).dj(2,1,:);
end

% -------------------------------------------------------------------------
% PLOT
% -------------------------------------------------------------------------
% Set parameters
if strcmp(Trial(itrial).motion,'AA') == 1 || strcmp(Trial(itrial).motion,'FE') == 1
   x_label = 'Humerothoracic angle (°)';
   X       = squeeze(Joint(1).Euler(1,2,:));
elseif strcmp(Trial(itrial).motion,'HFE') == 1
   x_label = 'Humerothoracic angle (°)';
   X       = squeeze(Joint(1).Euler(1,1,:));
elseif strcmp(Trial(itrial).motion,'IER') == 1
   x_label = 'Humerothoracic angle (°)';
   X       = squeeze(Joint(1).Euler(1,3,:));
elseif strcmp(Trial(itrial).motion,'VT') == 1
   x_label = 'Humerothoracic displacement (mm)';
   X       = squeeze(Joint(1).dj(2,1,:));
elseif strcmp(Trial(itrial).motion,'HC') == 1
   x_label = 'Humerothoracic displacement (mm)';
   X       = squeeze(Joint(1).dj(1,1,:));
end
% Create plots
if fig == 1
    for ijoint = 1:5
        figure(ijoint+1);
        sgtitle([Joint(ijoint).label,' (',Joint(ijoint).sequence,')']);
        subplot(1,2,1); hold on; grid on; box on;
        xlabel(x_label);
        ylabel('Rotation (°)');
        plot(X,squeeze(Joint(ijoint).Euler(1,1:3,:))');
        legend(Joint(ijoint).legend(1:3));
        subplot(1,2,2); hold on; grid on; box on;
        xlabel(x_label);
        ylabel('Displacement (mm)');
        plot(X,squeeze(Joint(ijoint).dj(1:3,1,:))');
        legend(Joint(ijoint).legend(4:6));
    end
end

% -------------------------------------------------------------------------
% STORE KINEMATICS IN CSV FILE
% -------------------------------------------------------------------------
if file == 1
    fileName  = ['../',Subject.id,'_',Subject.side,'_',Trial(itrial).motion,'_kinematics.csv'];
    T         = {x_label, ...
                 [Joint(1).label,' (',Joint(1).sequence,')'] [Joint(1).label,' (',Joint(1).sequence,')'] [Joint(1).label,' (',Joint(1).sequence,')'] [Joint(1).label,' (',Joint(1).sequence,')'] [Joint(1).label,' (',Joint(1).sequence,')'] [Joint(1).label,' (',Joint(1).sequence,')'], ...
                 [Joint(2).label,' (',Joint(2).sequence,')'] [Joint(2).label,' (',Joint(2).sequence,')'] [Joint(2).label,' (',Joint(2).sequence,')'] [Joint(2).label,' (',Joint(2).sequence,')'] [Joint(2).label,' (',Joint(2).sequence,')'] [Joint(2).label,' (',Joint(2).sequence,')'], ...
                 [Joint(3).label,' (',Joint(3).sequence,')'] [Joint(3).label,' (',Joint(3).sequence,')'] [Joint(3).label,' (',Joint(3).sequence,')'] [Joint(3).label,' (',Joint(3).sequence,')'] [Joint(3).label,' (',Joint(3).sequence,')'] [Joint(3).label,' (',Joint(3).sequence,')'], ...
                 [Joint(4).label,' (',Joint(4).sequence,')'] [Joint(4).label,' (',Joint(4).sequence,')'] [Joint(4).label,' (',Joint(4).sequence,')'] [Joint(4).label,' (',Joint(4).sequence,')'] [Joint(4).label,' (',Joint(4).sequence,')'] [Joint(4).label,' (',Joint(4).sequence,')'], ...
                 [Joint(5).label,' (',Joint(5).sequence,')'] [Joint(5).label,' (',Joint(5).sequence,')'] [Joint(5).label,' (',Joint(5).sequence,')'] [Joint(5).label,' (',Joint(5).sequence,')'] [Joint(5).label,' (',Joint(5).sequence,')'] [Joint(5).label,' (',Joint(5).sequence,')']; ...
                 '', ...
                 Joint(1).legend(1), Joint(1).legend(2), Joint(1).legend(3), Joint(1).legend(4), Joint(1).legend(5), Joint(1).legend(6), ...
                 Joint(2).legend(1), Joint(2).legend(2), Joint(2).legend(3), Joint(2).legend(4), Joint(2).legend(5), Joint(2).legend(6), ...
                 Joint(3).legend(1), Joint(3).legend(2), Joint(3).legend(3), Joint(3).legend(4), Joint(3).legend(5), Joint(3).legend(6), ...
                 Joint(4).legend(1), Joint(4).legend(2), Joint(4).legend(3), Joint(4).legend(4), Joint(4).legend(5), Joint(4).legend(6), ...
                 Joint(5).legend(1), Joint(5).legend(2), Joint(5).legend(3), Joint(5).legend(4), Joint(5).legend(5), Joint(5).legend(6); ...
                 };
    T2        = array2table(T);
    for iframe = 1:size(X,1)
        T2        = [T2; ...
                     {X(iframe), ...
                     squeeze(Joint(1).Euler(1,1,iframe)), squeeze(Joint(1).Euler(1,2,iframe)), squeeze(Joint(1).Euler(1,3,iframe)), squeeze(Joint(1).dj(1,1,iframe)), squeeze(Joint(1).dj(2,1,iframe)), squeeze(Joint(1).dj(3,1,iframe)), ...
                     squeeze(Joint(2).Euler(1,1,iframe)), squeeze(Joint(2).Euler(1,2,iframe)), squeeze(Joint(2).Euler(1,3,iframe)), squeeze(Joint(2).dj(1,1,iframe)), squeeze(Joint(2).dj(2,1,iframe)), squeeze(Joint(2).dj(3,1,iframe)), ...
                     squeeze(Joint(3).Euler(1,1,iframe)), squeeze(Joint(3).Euler(1,2,iframe)), squeeze(Joint(3).Euler(1,3,iframe)), squeeze(Joint(3).dj(1,1,iframe)), squeeze(Joint(3).dj(2,1,iframe)), squeeze(Joint(3).dj(3,1,iframe)), ...
                     squeeze(Joint(4).Euler(1,1,iframe)), squeeze(Joint(4).Euler(1,2,iframe)), squeeze(Joint(4).Euler(1,3,iframe)), squeeze(Joint(4).dj(1,1,iframe)), squeeze(Joint(4).dj(2,1,iframe)), squeeze(Joint(4).dj(3,1,iframe)), ...
                     squeeze(Joint(5).Euler(1,1,iframe)), squeeze(Joint(5).Euler(1,2,iframe)), squeeze(Joint(5).Euler(1,3,iframe)), squeeze(Joint(5).dj(1,1,iframe)), squeeze(Joint(5).dj(2,1,iframe)), squeeze(Joint(5).dj(3,1,iframe))}];
    end
    writetable(T2,fileName,'WriteVariableNames',0);
end