%   Get lidar detections
    query = (' ');
    if (ismember(1, LidarNums))
        query = [query 'select 1 as Lidar, TimeCs, ObjCnt, XV, YV, ZV, Sx, Sy, Sz, Rot from Lidar1ObjCalibratedCs'];
        query = [query ' where RunId = ' num2str(RunId)];    
        query = [query ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
    end
    if (ismember(2, LidarNums))    
        if (ismember(1, LidarNums))    
            query = [query ' union '];
        end
        query = [query 'select 2 as Lidar, TimeCs, ObjCnt, XV, YV, ZV, Sx, Sy, Sz, Rot from Lidar2ObjCalibratedCs'];
        query = [query ' where RunId = ' num2str(RunId)];    
        query = [query ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
    end
    query = [query ' order by Lidar, ObjCnt, TimeCs'];
    LidarDetections = fetch(conn, query);
    clear query

%   Cast lidar detections into the camera's axis system.    
    [LidarDetections.XTC, LidarDetections.YTC, LidarDetections.ZTC] = ...
        CastVehicleCoordstoCamera(LidarDetections.XV, LidarDetections.YV, LidarDetections.ZV, CameraExtCal);    
%   Eliminate any detection that is behind the camera or well outside of
%   the field of view.
    IsForward = (LidarDetections.ZTC > 0 & ...
                LidarDetections.XTC./LidarDetections.ZTC >= xpcmin & ...
                LidarDetections.XTC./LidarDetections.ZTC <= xpcmax & ...
                LidarDetections.YTC./LidarDetections.ZTC >= ypcmin & ...
                LidarDetections.YTC./LidarDetections.ZTC <= ypcmax);
    LidarDetections = LidarDetections(IsForward,:);
    clear IsForward
    
%   Get the row and column for each POV detection.    
    [LidarDetections.col, LidarDetections.row] = ...
        ReverseCameraIntCal(CameraIntCal, LidarDetections.XTC, LidarDetections.YTC, LidarDetections.ZTC);
    
%   Get bounding box corners.
    if (UseLidarBoundingBoxes == 1)
        nLD = height(LidarDetections);
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
                LidarDetections.XV + facx*LidarDetections.Sx.*cos(LidarDetections.Rot) - facy*LidarDetections.Sy.*sin(LidarDetections.Rot),...
                LidarDetections.YV + facx*LidarDetections.Sx.*sin(LidarDetections.Rot) + facy*LidarDetections.Sy.*cos(LidarDetections.Rot),...
                                          LidarDetections.ZV + facz*LidarDetections.Sz, CameraExtCal);     
            [LDVertexCols(:,m+1), LDVertexRows(:,m+1)] = ...
                ReverseCameraIntCal(CameraIntCal, LDVX, LDVY, LDVZ);...
            m = m + 1;
        end
        vars = {'LDVX', 'LDVY', 'LDVZ', 'm', 'nLD', 'facx', 'facy', 'facz'};
        clear (vars{:})
    end
   