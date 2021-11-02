%% Track creation of LiDAR objects using association and filtering
% We use the Munkres method to associate objects to tracks
% Then we use Multi-hypothesis EKF to form tracks
% Basic idea similar to https://www.thinkautonomous.ai/blog/?p=computer-vision-for-tracking

clear
clc
warning('off')

%% Basic SQL Querying GTTRAnalysis::LidarObjects data

conn_gttr4 = database('GTTR4CPU','','');
conn_analysis = database('GttrAnalysis','','');
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

            initTime = tripsC.TimeCs(1);
            finalTime = tripsC.TimeCs(numel(tripsC.TimeCs));
            minFrame = tripsC.Frame(1);
            maxFrame = tripsC.Frame(numel(tripsC.TimeCs));

            % Loop through the recorded lidar data, generate detections from the
            % current point cloud using the detector model and then process the
            % detections using the tracker.
            time = initTime;       % Start time
            prevTime = -1;
            delta_t = 0.05;               % Time step  Object detections at 20 Hz


            %Read the objects for the RunId
            %sqlStr1 = ' SELECT * FROM [GTTRAnalysis].[dbo].[Lidar1ObjCs] where runid = ';
            %sqlStr2 = '  and TimeCs >= 0 order by Vehicle, RunId, TimeCs, ObjCnt';
            sqlStr1 = sprintf(' SELECT * FROM [GTTR4CPU].[dbo].[Lidar%uObjCalibratedCs] where runid = %u', iLidarNum, runId);
            %sqlStr1 = sprintf('%s and abs(1 - 1/(1+SQUARE(((TimeCs-1435)/90))) - (30.06+X)/24.23) < .3 and TimeCs > 1300 and Y > 0 and Y < 2', sqlStr1);
            sqlStr2 = '  order by Vehicle, RunId, TimeCs, ObjCnt';
            
            sqlStr = [sqlStr1 sqlStr2];
            curs = exec(conn_gttr4,sqlStr);
            ObjectsC = fetch(conn_gttr4,sqlStr);
            Detections = table2struct(ObjectsC);
            
            % Initiate all tracks.
            currTracks = struct('ObjCnt', [], 'TrackId', [], 'state', [], 'cov', [], 'is_stale', [], 'stale_time', []);
            minFrame = max(minFrame, min(ObjectsC.Frame));
            maxFrame = min(maxFrame, max(ObjectsC.Frame));
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
            for frameId = minFrame:maxFrame
                % Update time
                %time = time + dT;
                for i = 1:numel(tripsC.TimeCs)
                   if (frameId == tripsC.Frame(i))
                       time = tripsC.TimeCs(i);
                       break;
                   end
                end

                % Load the detections array with objects that were previously
                % detected by the model zoo
                % for the first frame, by default, make the detections as
                % the track
                if (frameId == minFrame)
                    num_tracks = 1;
                    ind = find(ObjectsC.Frame == frameId);
                    for j = 1:numel(ind)
                        currTracks(num_tracks).ObjCnt = Detections(ind(j)).ObjCnt;
                        currTracks(num_tracks).TrackId = num_tracks;
                        currTracks(num_tracks).state = [Detections(ind(j)).XV,...
                                                        Detections(ind(j)).YV,...
                                                        Detections(ind(j)).ZV,...
                                                        Detections(ind(j)).Sx,...
                                                        Detections(ind(j)).Sy,...
                                                        Detections(ind(j)).Sz,...
                                                        Detections(ind(j)).Rot,...
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
                    curr_det_ind = find(ObjectsC.Frame == frameId);
                    
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