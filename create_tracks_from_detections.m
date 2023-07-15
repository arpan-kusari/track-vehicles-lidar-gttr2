%% Track creation of LiDAR objects using association and filtering
% We use the Munkres method to associate objects to tracks
% Then we use Multi-hypothesis EKF to form tracks
% Basic idea similar to https://www.thinkautonomous.ai/blog/?p=computer-vision-for-tracking

clear
clc
warning('off')

%% Basic SQL Querying GTTRAnalysis::LidarObjects data

conn_analysis = database('GttrV2Analysis','','');
delta_t = 0.05; % Time step object detections at 20 Hz
% select the runids to process
for runId=276:277
    for iLidarNum=1:1
        % read summary data for runId to find initTime and finalTime
        % sqlStr1 = 'SELECT min([TimeCs]) as StartTime,max([TimeCs]) as EndTime, min([Frame]) as MinFrame, max([Frame]) as MaxFrame FROM [GTTR4CPU].[dbo].[Lidar1Obj] where RunId = ';
        sqlStr1 = sprintf('SELECT [TimeCs], [Frame] FROM [GttrV2Analysis].[dbo].[LidarObjCalCsMapTh] where RunId = %u', runId);
        sqlStr2 = ' order by TimeCs, Frame';
        sqlStr = [sqlStr1 sqlStr2];
        curs = exec(conn_analysis,sqlStr); 
        tripsC = fetch(conn_analysis,sqlStr);
        
        isql = sprintf('DELETE FROM [GttrV2Analysis].[dbo].[LidarTrackCs] where Runid = %u', runId);
        curs = exec(conn_analysis,isql);
        
        if (numel(tripsC) > 0)
            %Read the objects for the RunId
            sqlStr1 = sprintf(' SELECT * FROM [GttrV2Analysis].[dbo].[LidarObjCalCsMapTh] where Runid = %u', runId);
            sqlStr2 = '  order by Vehicle, RunId, TimeCs, ObjCnt';
            
            sqlStr = [sqlStr1 sqlStr2];
            curs = exec(conn_analysis,sqlStr);
            ObjectsC = fetch(conn_analysis,sqlStr);
            Detections = table2struct(ObjectsC);
            
            % Initiate all tracks.
            currTracks = struct('ObjCnt', {}, 'TrackId', {}, 'state', {}, 'cov', {}, 'is_stale', {}, 'stale_time', {});

            
            unique_time = unique(ObjectsC.TimeCs);
            % Value of loss_GIOU = [0, 2)
            % From "Distance-IoU Loss: Faster and Better Learning for
            % Bounding Box Regression"
            % threshold about 5m distance between centroids of boxes
            costOfNonAssignment = 0.6;
            
            % State vector for Kalman box prediction
            % x = [cx, cy, cz, l, w, h, theta, vx, vy, vz, vl, vw, vh,
            % vtheta]
            cov_start = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, deg2rad(10), 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, deg2rad(10)]);
            % measurement matrix 
            meas_noise_mat = diag([0.2, 0.2, 0.2, 0.5, 0.5, 0.5, deg2rad(10)]);
            %stale observation limit
            num_stale = 5;
            global_track_id = 1;
            % Loop through the data
            for time_ind = 1:length(unique_time)
                % Update time
                curr_time = unique_time(time_ind);
                if curr_time < 0
                    continue
                end
                
                % Load the detections array with objects that were previously
                % detected by the model zoo
                % for the first frame, by default, make the detections as
                % the track
                if (size(currTracks, 1) == 0)
                    num_tracks = 1;
                    curr_ind = find(ObjectsC.TimeCs == curr_time);
                    for j = 1:numel(curr_ind)
                        if Detections(curr_ind(j)).OnRoadFlag == 0
                            continue
                        end
                        currTracks(num_tracks, 1).ObjCnt = Detections(curr_ind(j)).ObjCnt;
                        currTracks(num_tracks, 1).TrackId = global_track_id;
                        currTracks(num_tracks, 1).state = [Detections(curr_ind(j)).XTV,...
                                                        Detections(curr_ind(j)).YTV,...
                                                        Detections(curr_ind(j)).ZTV,...
                                                        Detections(curr_ind(j)).Sx,...
                                                        Detections(curr_ind(j)).Sy,...
                                                        Detections(curr_ind(j)).Sz,...
                                                        Detections(curr_ind(j)).RotV,...
                                                        0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]';
                        currTracks(num_tracks, 1).cov = cov_start;
                        currTracks(num_tracks, 1).is_stale = false;
                        currTracks(num_tracks, 1).stale_time = 0;
                        global_track_id = global_track_id + 1;
                        num_tracks = num_tracks + 1;
                        frameId = Detections(curr_ind(j)).Frame;
                        
                    end
%                     if any(vertcat(Detections(curr_ind).OnRoadflag) == 1)
%                         first_pass = false;
%                     end
                    prev_tracks = currTracks;
                else
                    for t = 1:size(currTracks, 1)
                        % run prediction step of kalman filter to get updated 
                        % state and covariance
                        [currTracks(t, 1).state, currTracks(t, 1).cov] = predict_kalman(currTracks(t, 1).state, currTracks(t, 1).cov, delta_t);
                    end
                    curr_det_ind = find(ObjectsC.TimeCs == curr_time);
                    if isempty(curr_det_ind)
                        % if no current detections present, make all tracks
                        % stale
                        num_tracks = size(currTracks, 1);
                        while t < num_tracks
                            if (currTracks(t, 1).is_stale == true && currTracks(t, 1).stale_time > delta_t*num_stale)
                                currTracks([t]) = [];
                                num_tracks = num_tracks - 1;
                                if size(currTracks, 2) == 0
                                    currTracks = struct('ObjCnt', {}, 'TrackId', {}, 'state', {}, 'cov', {}, 'is_stale', {}, 'stale_time', {});
                                end
                            else
                                currTracks(t, 1).is_stale = true;
                                currTracks(t, 1).stale_time = currTracks(t, 1).stale_time + delta_t;
                            end
                        end
                        continue
                    end
                    frameId = Detections(curr_det_ind(1)).Frame;
                    % we have to associate the detections to tracks
                    cost_matrix = zeros(size(currTracks, 1), numel(curr_det_ind));
                    for t = 1:size(currTracks,1)
                        for j = 1:numel(curr_det_ind)
                            cost = estimate_3D_GIOU(currTracks(t, 1), Detections(curr_det_ind(j)));
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
                        for ass_ind = 1:size(assignments, 1)
                            track_ind = assignments(ass_ind, 1);
                            detection_ind = curr_det_ind(assignments(ass_ind, 2));
                            % check if is_stale is True
                            % since there is an active detection matching
                            % the track, remove the is_stale flag
                            if(currTracks(track_ind, 1).is_stale == true)
                                currTracks(track_ind, 1).is_stale = false;
                                currTracks(track_ind, 1).stale_time = 0;
                            end
                            % filter step based on new measurement
                            meas = [Detections(detection_ind).XTV;...
                                    Detections(detection_ind).YTV;...
                                    Detections(detection_ind).ZTV;...
                                    Detections(detection_ind).Sx;...
                                    Detections(detection_ind).Sy;...
                                    Detections(detection_ind).Sz;...
                                    Detections(detection_ind).RotV];
                            del_state = meas - prev_tracks(track_ind).state(1:7);
                            if any(abs(del_state(1:3)) > 1)
                                disp('Out of bounds')
                            end
                            [state_est, cov_est] = update_kalman(currTracks(track_ind, 1).state, currTracks(track_ind, 1).cov, meas, meas_noise_mat);
                            currTracks(track_ind, 1).ObjCnt = Detections(detection_ind).ObjCnt;
                            % currTracks(track_ind, 1).TrackId = track_ind;
                            currTracks(track_ind, 1).state = state_est;
                            currTracks(track_ind, 1).cov = cov_est;
                        end
                    end
                    % check if there are unassignedTracks
                    if(~isempty(unassignedTracks))
                        for unass_ind = 1:size(unassignedTracks, 1)
                            % if there are, make them stale and start counting
                            % time
                            % if greater than time limit, then remove track
                            if (currTracks(unassignedTracks(unass_ind), 1).is_stale == true && currTracks(unassignedTracks(unass_ind), 1).stale_time > delta_t*num_stale)
                                currTracks([unassignedTracks(unass_ind)]) = [];
                                unassignedTracks(unass_ind+1:end, 1) = unassignedTracks(unass_ind+1:end, 1) - 1;
                                if size(currTracks, 2) == 0
                                    currTracks = struct('ObjCnt', {}, 'TrackId', {}, 'state', {}, 'cov', {}, 'is_stale', {}, 'stale_time', {});
                                end
                            else
                                currTracks(unassignedTracks(unass_ind), 1).is_stale = true;
                                currTracks(unassignedTracks(unass_ind), 1).stale_time = currTracks(unassignedTracks(unass_ind), 1).stale_time + delta_t;
                            end
                        end
                    end
                    % check if there are unassignedDetections
                    if(~isempty(unassignedDetections))
                        for unass_ind = 1:size(unassignedDetections, 1)
                            % if there are unassignedDetections, then
                            % create new track
                            detection_ind = curr_det_ind(unassignedDetections(unass_ind));
                            num_tracks = size(currTracks,1);
                            num_tracks = num_tracks + 1;
                            global_track_id = global_track_id + 1;
                            currTracks(num_tracks, 1).TrackId = global_track_id;
                            currTracks(num_tracks, 1).ObjCnt = Detections(detection_ind).ObjCnt;
                            currTracks(num_tracks, 1).state = [Detections(detection_ind).XTV,...
                                                               Detections(detection_ind).YTV,...
                                                               Detections(detection_ind).ZTV,...
                                                               Detections(detection_ind).Sx,...
                                                               Detections(detection_ind).Sy,...
                                                               Detections(detection_ind).Sz,...
                                                               Detections(detection_ind).RotV,...
                                                               0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]';
                            currTracks(num_tracks, 1).cov = cov_start;
                            currTracks(num_tracks, 1).is_stale = false;
                            currTracks(num_tracks, 1).stale_time = 0;
                        end
                    end
                    prev_tracks = currTracks;
                    for track_ind = 1:size(currTracks, 1)
                        % write the track to the table
                        isql = sprintf('INSERT INTO [GttrV2Analysis].[dbo].[LidarTrackCs] ');
                        isql = sprintf('%s ([Vehicle], [RunId], [TimeCs], [Frame], [ObjCnt]', isql );
                        isql = sprintf('%s , [Track], [X], [Y], [Z]', isql );
                        isql = sprintf('%s , [Sx], [Sy], [Sz], [Rot]) ', isql );
                        isql = sprintf('%s VALUES ( ', isql );
                        isql = sprintf('%s %u, ', isql, 1 );
                        isql = sprintf('%s %u, ', isql, runId );
                        isql = sprintf('%s %u, ', isql, curr_time );
                        isql = sprintf('%s %u, ', isql, frameId);
                        isql = sprintf('%s %u, ', isql, currTracks(track_ind, 1).ObjCnt);
                        isql = sprintf('%s %u, ', isql, currTracks(track_ind, 1).TrackId);
                        
                        % fprintf('Track id = %d\n', currTracks(t, 1).TrackId);
                        for state_ind = 1:6
                            isql = sprintf(' %s %f,', isql, currTracks(track_ind, 1).state(state_ind));
                        end
                        isql = sprintf(' %s %f)', isql, currTracks(track_ind, 1).state(7));
                        execute(conn_analysis,isql);
                        % run prediction step of kalman filter to get updated 
                        % state and covariance
                        % [currTracks(t).state, currTracks(t).cov] = predict_kalman(currTracks(t).state, currTracks(t).cov, delta_t);
                    end
                    fprintf('Wrote tracks fron time = %d\n', curr_time);
                end
                
            end
        end
    end
end
close(conn_analysis);