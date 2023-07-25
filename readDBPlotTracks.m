% Basic SQL Querying GTTRAnalysis::LidarObjects data
%
%

clear
clc
warning('off')

conn = database('GttrAnalysis','','');
conn.Message;
tic


% Set random seed to generate reproducible results.
S = rng(2018);


% select the runids to process
for i=260:260
    
%     sqlStr1 = sprintf(' SELECT * FROM [GTTR4CPUv2].[dbo].[InsOutCorrectedCs] where Runid = %u', runId);
%     sqlStr2 = '  order by RunId, TimeCs';
% 
%     sqlStr = [sqlStr1 sqlStr2];
%     curs = exec(conn_gttr4,sqlStr);
%     EgoData = fetch(conn_gttr4,sqlStr);
%     %% for this project only
%     % Pitch is negative of pitch
%     % Roll needs 180 deg added
%     EgoData.Pitch = -EgoData.Pitch;
%     EgoData.Roll = deg2rad(EgoData.Roll + 180);
%     EgoData.Roll = rad2deg(atan2(sin(EgoData.Roll), cos(EgoData.Roll)));
%     EgoDataStruct = table2struct(EgoData);


    % read summary data for runId to find initTime and finalTime
    sqlStr1 = 'SELECT min([TimeCs]) as StartTime,max([TimeCs]) as EndTime, min([Frame]) as MinFrame, max([Frame]) as MaxFrame FROM [Gttr4CPUv2].[dbo].[LidarObjCalibratedCs] where RunId = ';

    runId = i;
    sqlStr = [sqlStr1 num2str(runId)];
    %curs = exec(conn,sqlStr); %#ok<NASGU>
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
    sqlStr1 = sprintf(' SELECT * FROM [Gttr4CPUv2].[dbo].[LidarObjCalibratedCs] where Runid = %u', runId);
    % sqlStr1 = sprintf('%s and abs(1 - 1/(1+SQUARE(((TimeCs-1435)/90))) - (30.06+X)/24.23) < .3 and TimeCs > 1300 and Y > 0 and Y < 2', sqlStr1);
    sqlStr2 = '  order by Vehicle, RunId, TimeCs, ObjCnt';

    sqlStr = [sqlStr1 sqlStr2];
    %curs = exec(conn,sqlStr);
    ObjectsC = fetch(conn,sqlStr);
    Detections = table2struct(ObjectsC);
    fieldNames = fieldnames(Detections);

    %Display the attributes and values using a loop
%     fprintf('Attributes and Values of the Structure:\n');
%     for h = 1:3
%         for k = 1:numel(fieldNames)
%             attributeName = fieldNames{k};
%             attributeValue = Detections(h).(attributeName);
%             
%             fprintf('%s: ', attributeName);
%             disp(attributeValue);
%         end
    % end
    %disp(cell2mat({Detections.TimeGps}));

    % Read the Tracks from Lidar%uTrackCs
    % read summary data for runId to find initTime and finalTime
    runId = i;
    sqlStr = sprintf('SELECT [Vehicle],[RunId],[TimeCs] as time, [Frame], [Track] as TrackId,');
    sqlStr = sprintf('%s [X] as St1, [Y] as St2, [Z] as St3,', sqlStr);
    sqlStr = sprintf('%s [Sx] as St4, [Sy] as St5, [Sz] as St6, [Rot] as St7', sqlStr);
    sqlStr = sprintf('%s FROM [GttrV2Analysis].[dbo].[LidarTrackCs] where RunId = %d', sqlStr, runId);
    sqlStr = sprintf('%s Order By Vehicle, RunId, time, TrackId', sqlStr);
    
    %curs = exec(conn,sqlStr); 
    DBHistoryC = fetch(conn,sqlStr);
    
    Lidar_folder = "\\tri-gt1\Gttr4CPUv2\00001\Main\" + "001-00" + runId + "\";
    
    timefile = memmapfile(Lidar_folder + "timeFile", 'Format', {'uint64', [1,1], 'TimeGps'; 'uint32', [1, 1], 'Count'}, 'Repeat', Inf);

    Data = timefile.Data;
    TimeGpsArr = double([Data.TimeGps]);
    Count = double([Data.Count]);
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
    
    output_filename = 'images/track_result.gif';
    count = 1;
    for frameId = minFrame:maxFrame
        ind_d = find([Detections.Frame] == frameId);
        ind_t = find([currTracks.Frame] == frameId);

        if isempty(ind_d) || isempty(ind_t)
            continue;
        end

%         disp("hi");
%         disp(ind_d);
%         disp(size(cell2mat({Detections(ind_d).TimeGps})));

        TimeGpsDetectionArr = cell2mat({Detections(ind_d).TimeGps});
        disp(size(unique(TimeGpsDetectionArr))); %always 1x1? if not, what would we subtract from TimeGpsArr in the next line?
        [~, ind_pc] = min(abs(TimeGpsArr - TimeGpsDetectionArr(1)));
        las = lasFileReader(Lidar_folder + sprintf('%06d', Count(ind_pc)) + ".laz");
        pc = readPointCloud(las);
        pcshow(pc);
%        plot(0, 0, 'bo', 'MarkerSize', 20);
        
        for j = 1:size(ind_d, 1)
            id = sprintf('O%d', Detections(ind_d(j)).ObjCnt);
            rgb = [rand, rand, rand];
            draw_rectangle(Detections(ind_d(j)).XTV,...
                           Detections(ind_d(j)).YTV,...
                           Detections(ind_d(j)).Sx,...
                           Detections(ind_d(j)).Sy,...
                           Detections(ind_d(j)).RotV,...
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
        f = getframe(fig); 
        im = frame2im(f); 
        [imind,cm] = rgb2ind(im,256); 
        if count == 1
            imwrite(imind,cm,output_filename, 'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,output_filename,'gif','WriteMode','append'); 
        end
        pause(0.1);
        clf(fig);
        count = count + 1;
    end
end