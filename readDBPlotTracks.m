% Basic SQL Querying GTTRAnalysis::LidarObjects data
%
%

clear
clc
warning('off')

conn = database('GTTRAnalysis','','');
tic


% Set random seed to generate reproducible results.
S = rng(2018);


% select the runids to process
for i=222:222

    % read summary data for runId to find initTime and finalTime
    sqlStr1 = 'SELECT min([TimeCs]) as StartTime,max([TimeCs]) as EndTime, min([Frame]) as MinFrame, max([Frame]) as MaxFrame FROM [GTTRAnalysis].[dbo].[Lidar1ObjCs] where RunId = ';

    runId = i;
    sqlStr = [sqlStr1 num2str(runId)];
    curs = exec(conn,sqlStr); %#ok<NASGU>
    tripsC = fetch(conn,sqlStr);
    initTime = tripsC.StartTime;
    finalTime = tripsC.EndTime;
    minFrame = tripsC.MinFrame;
    maxFrame = tripsC.MaxFrame;

    % Loop through the recorded lidar data, generate detections from the
    % current point cloud using the detector model and then process the
    % detections using the tracker.
    time = initTime;       % Start time
    dT = 5;                % Time step  Object detections at 20 Hz
    
    %Read the objects for the RunId
    %sqlStr1 = ' SELECT * FROM [GTTRAnalysis].[dbo].[Lidar1ObjCs] where runid = ';
    %sqlStr2 = '  and TimeCs >= 0 order by Vehicle, RunId, TimeCs, ObjCnt';
    sqlStr1 = sprintf(' SELECT * FROM [GTTRAnalysis].[dbo].[Lidar%uObjCs] where runid = %u', 1, runId);
    sqlStr1 = sprintf('%s and abs(1 - 1/(1+SQUARE(((TimeCs-1435)/90))) - (30.06+X)/24.23) < .3 and TimeCs > 1300 and Y > 0 and Y < 2', sqlStr1);
    sqlStr2 = '  order by Vehicle, RunId, TimeCs, ObjCnt';

    sqlStr = [sqlStr1 sqlStr2];
    curs = exec(conn,sqlStr);
    ObjectsC = fetch(conn,sqlStr);
    Detections = table2struct(ObjectsC);


    % Read the Tracks from Lidar%uTrackCs
    % read summary data for runId to find initTime and finalTime
    runId = i;
    sqlStr = sprintf('SELECT [Vehicle],[RunId],[TimeCs] as time, [Frame], [Track] as TrackId,');
    sqlStr = sprintf('%s [X] as St1, [Y] as St2, [Z] as St3,', sqlStr);
    sqlStr = sprintf('%s [Sx] as St4, [Sy] as St5, [Sz] as St6, [Rot] as St7', sqlStr);
    sqlStr = sprintf('%s FROM [GTTRAnalysis].[dbo].[Lidar1TrackCs] where RunId = %d', sqlStr, runId);
    sqlStr = sprintf('%s Order By Vehicle, RunId, time, TrackId', sqlStr);
    
    curs = exec(conn,sqlStr); 
    DBHistoryC = fetch(conn,sqlStr);

    % Loop through the cursor, to load the history.    
    % Initiate all tracks.
    % currTracks = struct('TrackId', [], 'Frame', [], 'X', [], 'Y', [], 'Z', [], 'Sx', [], 'Sy', [], 'Sz', [], 'Rot', []);
    currTracks = struct([]);  
    for curLoop = 1:numel(DBHistoryC.Vehicle)
        track.TrackId   = DBHistoryC.TrackId(curLoop);
        track.Frame     = DBHistoryC.Frame(curLoop);
        track.X         = DBHistoryC.St1(curLoop);
        track.Y  = DBHistoryC.St2(curLoop);
        track.Z  = DBHistoryC.St3(curLoop);
        track.Sx  = DBHistoryC.St4(curLoop);
        track.Sy  = DBHistoryC.St5(curLoop);
        track.Sz  = DBHistoryC.St6(curLoop);
        track.Rot  = DBHistoryC.St7(curLoop);
        currTracks = [currTracks; track];
    end
    % Loop through time, and display the tracks on screen 
    fig = figure(1);
    
    unique_tracks = unique([currTracks.TrackId]);
    rgb_tracks = rand(size(unique_tracks, 2), 3);
    all_tracks_X = [currTracks.X];
    all_tracks_Y = [currTracks.Y];
    min_x = min([currTracks.X]) - 20;
    max_x = max([max([currTracks.X]) + 20, 20]);
    min_y = min([currTracks.Y]) - 20;
    max_y = max([max([currTracks.Y]) + 20, 20]);
    
    for frameId = minFrame:maxFrame
        ind_d = find([Detections.Frame] == frameId);
        ind_t = find([currTracks.Frame] == frameId);
        if ~isempty(ind_d) && ~isempty(ind_t)
            for j = 1:size(ind_d, 1)
                id = sprintf('O%d', Detections(ind_d(j)).ObjCnt);
                rgb = [rand, rand, rand];
                draw_rectangle(Detections(ind_d(j)).X,...
                               Detections(ind_d(j)).Y,...
                               Detections(ind_d(j)).Sx,...
                               Detections(ind_d(j)).Sy,...
                               Detections(ind_d(j)).Rot,...
                               rgb,... 
                               id);
                axis([min_x max_x min_y max_y])
            end
            for j = 1:size(ind_t, 1)
                id = sprintf('%d', currTracks(ind_t(j)).TrackId);
                unique_ind = find(currTracks(ind_t(j)).TrackId == unique_tracks);
                rgb = rgb_tracks(unique_ind, :);
                draw_rectangle(currTracks(ind_t(j)).X,...
                               currTracks(ind_t(j)).Y,...
                               currTracks(ind_t(j)).Sx,...
                               currTracks(ind_t(j)).Sy,...
                               currTracks(ind_t(j)).Rot,...
                               rgb,... 
                               id);
                axis([min_x max_x min_y max_y])
            end
            pause(0.1);
            clf(fig);
        end
    end
end