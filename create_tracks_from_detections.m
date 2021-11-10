%% Track creation of LiDAR objects using association and filtering
% We use the Munkres method to associate objects to tracks
% Then we use Multi-hypothesis EKF to form tracks
% Basic idea similar to https://www.thinkautonomous.ai/blog/?p=computer-vision-for-tracking

clear
clc
warning('off')

%% Basic SQL Querying GTTRAnalysis::LidarObjects data

conn_gttr4 = database('GTTR4CPU','','');
conn_analysis = database('GTTRAnalysis','','');
delta_t = 0.05; % Time step object detections at 20 Hz
% select the runids to process
for runId=448:448
    for iLidarNum=1:2
        % read summary data for runId to find initTime and finalTime
        % sqlStr1 = 'SELECT min([TimeCs]) as StartTime,max([TimeCs]) as EndTime, min([Frame]) as MinFrame, max([Frame]) as MaxFrame FROM [GTTR4CPU].[dbo].[Lidar1Obj] where RunId = ';
        sqlStr1 = sprintf('SELECT [TimeCs], [Frame] FROM [GTTR4CPU].[dbo].[Lidar%uObjCalibratedCs] where RunId = ', iLidarNum);
        sqlStr2 = ' order by TimeCs, Frame';
        sqlStr = [sqlStr1 num2str(runId) sqlStr2];
        curs = exec(conn_gttr4,sqlStr); 
        tripsC = fetch(conn_gttr4,sqlStr);
        if (numel(tripsC) > 0)
            %Read the objects for the RunId
            sqlStr1 = sprintf(' SELECT * FROM [GTTR4CPU].[dbo].[Lidar%uObjCalibratedCs] where runid = %u', iLidarNum, runId);
            sqlStr2 = '  order by Vehicle, RunId, TimeCs, ObjCnt';
            
            sqlStr = [sqlStr1 sqlStr2];
            curs = exec(conn_gttr4,sqlStr);
            ObjectsC = fetch(conn_gttr4,sqlStr);
            Detections = table2struct(ObjectsC);
            
            % Initiate all tracks.
            currTracks = struct('ObjCnt', [], 'TrackId', [], 'state', [], 'cov', [], 'is_stale', [], 'stale_time', []);

            
            unique_time = unique(ObjectsC.TimeCs);
            % Value of loss_GIOU = [0, 2)
            % From "Distance-IoU Loss: Faster and Better Learning for
            % Bounding Box Regression"
            % TODO: Have to check the threshold
            costOfNonAssignment = 2;
            
            % State vector for Kalman box prediction
            % x = [cx, cy, cz, l, w, h, theta, vx, vy, vz, vl, vw, vh,
            % vtheta]
            cov_start = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, deg2rad(10), 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, deg2rad(10)]);
            % measurement matrix 
            meas_noise_mat = diag([0.2, 0.2, 0.2, 0.5, 0.5, 0.5, deg2rad(10)]);
            %stale observation limit
            num_stale = 10;
            % Loop through the data
            for ind = 1:length(unique_time)
                % Update time
                curr_time = unique_time(ind);
                % Load the detections array with objects that were previously
                % detected by the model zoo
                % for the first frame, by default, make the detections as
                % the track
                if (ind == 1)
                    num_tracks = 1;
                    curr_ind = find(ObjectsC.TimeCs == curr_time);
                    for j = 1:numel(ind)
                        currTracks(num_tracks).ObjCnt = Detections(curr_ind(j)).ObjCnt;
                        currTracks(num_tracks).TrackId = num_tracks;
                        currTracks(num_tracks).state = [Detections(curr_ind(j)).XV,...
                                                        Detections(curr_ind(j)).YV,...
                                                        Detections(curr_ind(j)).ZV,...
                                                        Detections(curr_ind(j)).Sx,...
                                                        Detections(curr_ind(j)).Sy,...
                                                        Detections(curr_ind(j)).Sz,...
                                                        Detections(curr_ind(j)).Rot,...
                                                        0, 0, 0, 0, 0, 0, 0]';
                        currTracks(num_tracks).cov = cov_start;
                        currTracks(num_tracks).is_stale = false;
                        currTracks(num_tracks).stale_time = 0;
                        num_tracks = num_tracks + 1;
                    end
                else
                    for t = 1:size(currTracks, 1)
                        % run prediction step of kalman filter to get updated 
                        % state and covariance
                        [currTracks(t).state, currTracks(t).cov] = predict_kalman(currTracks(t).state, currTracks(t).cov, delta_t);
                    end
                    curr_det_ind = find(ObjectsC.TimeCs == curr_time);
                    
                    % we have to associate the detections to tracks
                    cost_matrix = zeros(size(currTracks, 1), numel(curr_det_ind));
                    for t = 1:size(currTracks,1)
                        for j = 1:numel(curr_det_ind)
                            cost = estimate_3D_GIOU(currTracks(t), Detections(curr_det_ind(j)));
                            cost_matrix(t, j) = cost;
                        end
                    end
                    [assignments,...
                     unassignedTracks,...
                     unassignedDetections] = assignDetectionsToTracks(cost_matrix, costOfNonAssignment);
                    % check if there are assignments
                    if(~isempty(assignments))
                        % go through each of the state and detection
                        % assignment pair
                        for ind = 1:size(assignments, 1)
                            track_ind = assignments(ind, 1);
                            detection_ind = curr_det_ind(assignments(ind, 2));
                            % check if is_stale is True
                            % since there is an active detection matching
                            % the track, remove the is_stale flag
                            if(currTracks(track_ind).is_stale == true)
                                currTracks(track_ind).is_stale = false;
                                currTracks(track_ind).stale_time = 0;
                            end
                            % filter step based on new measurement
                            meas = [Detections(detection_ind).XV;...
                                    Detections(detection_ind).YV;...
                                    Detections(detection_ind).ZV;...
                                    Detections(detection_ind).Sx;...
                                    Detections(detection_ind).Sy;...
                                    Detections(detection_ind).Sz;...
                                    Detections(detection_ind).Rot];
                            [state_est, cov_est] = update_kalman(currTracks(track_ind).state, currTracks(track_ind).cov, meas, meas_noise_mat);
                            currTracks(track_ind).ObjCnt = Detections(detection_ind).ObjCnt;
                            currTracks(track_ind).state = state_est;
                            currTracks(track_ind).cov = cov_est;
                        end
                    end
                    % check if there are unassignedTracks
                    if(~isempty(unassignedTracks))
                        for ind = 1:size(unassignedTracks, 1)
                            % if there are, make them stale and start counting
                            % time
                            % if greater than time limit, then remove track
                            if (currTracks(unassignedTracks(ind)).is_stale == true && currTracks(unassignedTracks(ind)).stale_time > delta_t*num_stale)
                                currTracks(unassignedTracks(ind)) = [];
                            end
                            currTracks(unassignedTracks(ind)).is_stale = true;
                            currTracks(unassignedTracks(ind)).stale_time = currTracks(unassignedTracks(ind)).stale_time + delta_t;
                        end
                    end
                    % check if there are unassignedDetections
                    if(~isempty(unassignedDetections))
                        for ind = 1:size(unassignedDetections, 1)
                            % if there are unassignedDetections, then
                            % create new track
                            detection_ind = unassignedDetections(ind);
                            num_tracks = size(currTracks,1);
                            num_tracks = num_tracks + 1;
                            currTracks(num_tracks).TrackId = num_tracks;
                            currTracks(num_tracks).state = [Detections(detection_ind).XV,...
                                                            Detections(detection_ind).YV,...
                                                            Detections(detection_ind).ZV,...
                                                            Detections(detection_ind).Sx,...
                                                            Detections(detection_ind).Sy,...
                                                            Detections(detection_ind).Sz,...
                                                            Detections(detection_ind).Rot,...
                                                            0, 0, 0, 0, 0, 0, 0]';
                            currTracks(num_tracks).cov = cov_start;
                            currTracks(num_tracks).is_stale = false;
                            currTracks(num_tracks).stale_time = 0;
                        end
                    end
                    
                    for t = 1:size(currTracks, 1)
                        % write the track to the table
                        isql = sprintf('INSERT INTO [GTTRAnalysis].[dbo].[Lidar%uTrackCs] ', iLidarNum);
                        isql = sprintf('%s ([Vehicle], [RunId], [TimeCs], [Frame], [ObjCnt]', isql );
                        isql = sprintf('%s , [Track], [X], [Y], [Z]', isql );
                        isql = sprintf('%s , [Sx], [Sy], [Sz], [Rot]) ', isql );
                        isql = sprintf('%s VALUES ( ', isql );
                        isql = sprintf('%s %u, ', isql, 1 );
                        isql = sprintf('%s %u, ', isql, runId );
                        isql = sprintf('%s %u, ', isql, time );
                        isql = sprintf('%s %u, ', isql, frameId);
                        isql = sprintf('%s %u, ', isql, currTracks(t).ObjCnt);
                        isql = sprintf('%s %u, ', isql, currTracks(t).TrackId);
                        for state_ind = 1:6
                            isql = sprintf(' %s %f,', isql, currTracks(t).state(state_ind));
                        end
                        isql = sprintf(' %s %f)', isql, currTracks(t).state(7));
 
                        curs = exec(conn_analysis,isql);
                        % run prediction step of kalman filter to get updated 
                        % state and covariance
                        % [currTracks(t).state, currTracks(t).cov] = predict_kalman(currTracks(t).state, currTracks(t).cov, delta_t);
                    end
                    fprintf('Wrote tracks from frame id = %d\n', frameId);
                end
                
            end
        end
    end
end