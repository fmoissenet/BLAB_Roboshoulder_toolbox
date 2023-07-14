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
% Dependencies : - Biomechanical ToolKit: https://github.com/Biomechanical-ToolKit              
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

function Trial = LoadMotionFile(Subject,Static,Trial,c3dFile,condition,motion,itrial)

% Initialise Trial structure
Trial(itrial).file      = c3dFile;
Trial(itrial).condition = condition;
Trial(itrial).motion    = motion;
btkFile                 = btkReadAcquisition(c3dFile);
Trial(itrial).fmarker   = btkGetPointFrequency(btkFile);
Marker                  = btkGetMarkers(btkFile);
Event                   = btkGetEvents(btkFile);
if strcmp(motion,'Static')
    timeRange           = [fix(Event.AA(2)*Trial(itrial).fmarker);...
                           fix(Event.AA(2)*Trial(itrial).fmarker)+1]; % Anatomical position defined during abduction at low position (first cycle)
else 
    timeRange           = [fix(Event.(motion)(1)*Trial(itrial).fmarker);... % Use all of the three recorded cycles
                           fix(Event.(motion)(7)*Trial(itrial).fmarker)];
end
Trial(itrial).n0        = 1;
Trial(itrial).n1        = timeRange(2)-timeRange(1)+1;
Trial(itrial).Marker    = [];
% Set markerset    
markerSet = {['c',Subject.side,'HUM01'],['c',Subject.side,'HUM02'],['c',Subject.side,'HUM03'],['c',Subject.side,'HUM04'],...
             ['c',Subject.side,'SCA01'],['c',Subject.side,'SCA02'],['c',Subject.side,'SCA03'],['c',Subject.side,'SCA04'],...
             ['c',Subject.side,'CLA01'],['c',Subject.side,'CLA02'],['c',Subject.side,'CLA03'],['c',Subject.side,'CLA04'],...
             'cTHO01','cTHO02','cTHO03','cTHO04'}; 
% Load and process marker trajectories
if isempty(Static) % Special case for static
    kmarker = 0;
    for imarker = 1:length(markerSet)
        if isfield(Marker,markerSet{imarker})
            kmarker = kmarker+1;
            Trial(itrial).Marker(kmarker).Trajectory.raw = permute(Marker.(markerSet{imarker})(timeRange(1):timeRange(2),:),[2,3,1]);
            % Convert mm to m
            Trial(itrial).Marker(kmarker).Trajectory.processed = Trial(itrial).Marker(kmarker).Trajectory.raw*1e-3;
            % Replace [0 0 0] by NaN
            for iframe = 1:Trial(itrial).n1
                if Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe) == [0 0 0]
                   Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe) = nan(3,1,1);
                else
                   Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe) = Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe);
                end
            end   
            % Compute mean marker position
            if isempty(Trial(itrial).Marker(kmarker).Trajectory.raw)
                Trial(itrial).Marker(kmarker).Trajectory.processed = [];
            else
                Trial(itrial).Marker(kmarker).Trajectory.processed = mean(Trial(itrial).Marker(kmarker).Trajectory.processed,3,'omitnan');
            end
            Trial(itrial).n0 = 1;
            Trial(itrial).n1 = 1;  
        end
    end
else
    kmarker = 0;
    for imarker = 1:length(markerSet)
        if isfield(Marker,markerSet{imarker})
            kmarker = kmarker+1;
            Trial(itrial).Marker(kmarker).Trajectory.raw = permute(Marker.(markerSet{imarker})(timeRange(1):timeRange(2),:),[2,3,1]);
            % Convert mm to m
            Trial.Marker(kmarker).Trajectory.processed = Trial(itrial).Marker(kmarker).Trajectory.raw*1e-3;
            % Replace [0 0 0] by NaN
            for iframe = 1:Trial(itrial).n1
                if Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe) == [0 0 0]
                   Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe) = nan(3,1,1);
                else
                   Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe) = Trial(itrial).Marker(kmarker).Trajectory.processed(:,:,iframe);
                end
            end   
        end
    end
    % Fill gaps (intercorrelation by PCA: https://doi.org/10.1371/journal.pone.0152616)
    tMarker = [];
    for imarker = 1:size(Trial(itrial).Marker,2)
        if ~isempty(Trial(itrial).Marker(imarker).Trajectory.raw)
            tMarker = [tMarker permute(Trial(itrial).Marker(imarker).Trajectory.processed,[3,1,2])];
        end
    end
    tMarker = PredictMissingMarkers(tMarker,'Algorithm',2);
    k = 0;
    for imarker = 1:size(Trial(itrial).Marker,2)
        if ~isempty(Trial(itrial).Marker(imarker).Trajectory.raw)
            k = k+1;
            Trial(itrial).Marker(imarker).Trajectory.processed = permute(tMarker(:,(3*k)-2:3*k),[2,3,1]);
        end
    end
    clear k tMarker;
    % Smooth trajectories (moving average)
    method = 'movmean';
    window  = 200; % frames
    for imarker = 1:size(Trial(itrial).Marker,2)
        Trial(itrial).Marker(imarker).Trajectory.processed = permute(smoothdata(permute(Trial(itrial).Marker(imarker).Trajectory.processed,[3,1,2]),method,window),[2,3,1]);
    end
end