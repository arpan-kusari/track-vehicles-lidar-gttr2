function [CameraExtCal] = GetExtrinsicCalParameters(CameraNum, CalibrationId)
%   This function queries for camera extrinsic calibration parameters.

%   Establish communication with the databases.
    conn = database('GTTR4CPU',' ',' ');

%   Get camera extrinsic calibration parameters.
    query = ('select * from CameraExtrinsicCals ');
    query = [query 'where Camera = ' num2str(CameraNum)];
    query = [query ' and CalibrationId = ' num2str(CalibrationId)];
    CameraExtCal = fetch(conn,query);
    
%   Shuffle to more convenient notation.
    ncal = height(CameraExtCal);
    if (ncal < 1)
        warning('No intrinsic cal values.');
        return
    elseif (ncal > 1)
        warning('Too many intrinsic cals.');
        return
    end
