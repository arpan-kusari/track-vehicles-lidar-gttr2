%   MUST INTERPOLATE LIDAR DETECTIONS TO THE NEAREST 10 CS!!!!!!!!
%   RIGHT NOW WE'RE SEEING DOUBLE.
%   Get lidar detections
    query = (' ');
    if (ismember(1, LidarNums))
        query = [query 'select 1 as Lidar, TimeCs, ObjCnt, Track, X, Y, Z, Sx, Sy, Sz, Rot from Lidar1TrackCs'];
        query = [query ' where RunId = ' num2str(RunId)];    
        query = [query ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
    end
    if (ismember(2, LidarNums))    
        if (ismember(1, LidarNums))    
            query = [query ' union '];
        end
        query = [query 'select 2 as Lidar, TimeCs, ObjCnt, Track, X, Y, Z, Sx, Sy, Sz, Rot from Lidar2TrackCs'];
        query = [query ' where RunId = ' num2str(RunId)];    
        query = [query ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
    end
    query = [query ' order by Lidar, Track, TimeCs'];
    LidarTracks = fetch(conna, query);
    clear query

%   Cast lidar detections into the camera's axis system.    
    [LidarTracks.XTC, LidarTracks.YTC, LidarTracks.ZTC] = ...
        CastVehicleCoordstoCamera(LidarTracks.X, LidarTracks.Y, LidarTracks.Z, CameraExtCal);    
%   Eliminate any detection that is behind the camera or well outside of
%   the field of view.
    IsForward = (LidarTracks.ZTC > 0 & ...
                LidarTracks.XTC./LidarTracks.ZTC >= xpcmin & ...
                LidarTracks.XTC./LidarTracks.ZTC <= xpcmax & ...
                LidarTracks.YTC./LidarTracks.ZTC >= ypcmin & ...
                LidarTracks.YTC./LidarTracks.ZTC <= ypcmax);
    LidarTracks = LidarTracks(IsForward,:);
    clear IsForward
    
%   Get the row and column for each POV detection.    
    [LidarTracks.col, LidarTracks.row] = ...
        ReverseCameraIntCal(CameraIntCal, LidarTracks.XTC, LidarTracks.YTC, LidarTracks.ZTC);
    
%   Get bounding box corners.
    if (UseLidarBoundingBoxes == 1)
        nLD = height(LidarTracks);
        LDVertexCols = zeros(nLD, 8); LDVertexRows = zeros(nLD, 8);
        LDVX = zeros(nLD, 1); LDVY = zeros(nLD, 1); LDVZ = zeros(nLD, 1);
        m = 0;
        while (m < 8)
            facx = ((-1)^m)/2.;
            facy = ((-1)^((m-mod(m,2))/2))/2.;
            facz = ((-1)^((m-mod(m,4))/4))/2.;
%           THIS IS LOUSY CODING, AND INFORRECT KINEMATICS. 
%           IT IS USING UNCALIBRATED VERSIONS OF Rot, Sx, Sy, and Sz. 
%           IT ONLY LOOKS VAGUELY CORRECT, BECAUSE THE SENSORS HAVE YAW
%           NEAR 180 DEGRESS, AND THERE IS NOT MUCH PITCH AND ROLL.
%           STRICTLY, THE BOXES SHOULD BE RESOLVED FROM THE SENSOR
%           COORDINATES TO VEHICLE COORDINATES BEFORE PROJECTION.
            [LDVX, LDVY, LDVZ] = ...
                CastVehicleCoordstoCamera(...
                LidarTracks.XV + facx*LidarTracks.Sx.*cos(LidarTracks.Rot) - facy*LidarTracks.Sy.*sin(LidarTracks.Rot),...
                LidarTracks.YV + facx*LidarTracks.Sx.*sin(LidarTracks.Rot) + facy*LidarTracks.Sy.*cos(LidarTracks.Rot),...
                                          LidarTracks.ZV + facz*LidarTracks.Sz, CameraExtCal);     
            [LDVertexCols(:,m+1), LDVertexRows(:,m+1)] = ...
                ReverseCameraIntCal(CameraIntCal, LDVX, LDVY, LDVZ);...
            m = m + 1;
        end
        vars = {'LDVX', 'LDVY', 'LDVZ', 'm', 'nLD', 'facx', 'facy', 'facz'};
        clear (vars{:})
    end
   