function [CameraIntCal] = GetIntrinsicCalParameters(CameraNum, CalibrationId)
%   This function queries for camera intrinsic calibration parameters.

%   Establish communication with the databases.
    conn = database('GTTR4CPU',' ',' ');

%   Get camera intrinsic calibration parameters.
    query = ('select * from CameraIntrinsicCals ');
    query = [query 'where Camera = ' num2str(CameraNum)];
    query = [query ' and CalibrationId = ' num2str(CalibrationId)];
    CameraIntCal = fetch(conn,query);
    
%   Make sure there's just one set.
    ncal = height(CameraIntCal);
    if (ncal < 1)
        warning('No intrinsic cal values.');
        return
    elseif (ncal > 1)
        warning('Too many intrinsic cals.');
        return
    end
    
%   Add field of view limits to the CameraIntCal table.
    [CameraIntCal.xpcmin(1), CameraIntCal.xpcmax(1), CameraIntCal.ypcmin(1), CameraIntCal.ypcmax(1)] = ...
        GetProjectionCoordLimits(CameraIntCal);
