function [IntCalNum, ExtCalNum] = GetVidCalNumsbyRunId(RunId, CameraNum)
%   This function seeks out the intrinsic calibration ID and extrinsic
%   calibration ID using run number. 

%   Establish communication with the databases.
    conn = database('GTTR4CPU',' ',' ');

%   Query.
    query = ('select distinct CalibrationIdIc, CalibrationIdEc');
    query = [query ' from V51ObjectsCalibrated'];
    query = [query ' where RunId = ' num2str(RunId)];
    query = [query ' and Camera = ' num2str(CameraNum)];
    CalIds = fetch(conn,query);
    ncal = height(CalIds);
    if (ncal < 1)
        warning('Could not get video cal numbers.');
        return
    elseif (ncal > 1)
        warning('More than 1 set of video cal numbers.');        
    end
    if (isnan(CalIds.CalibrationIdEc(1)))
        warning('Null extrinsic calibration ID.');        
        CalIds.CalibrationIdEc(1) = 1;
    end
    if (isnan(CalIds.CalibrationIdIc(1)))
        warning('Null intrinsic calibration ID.');        
        CalIds.CalibrationIdIc(1) = 1;
    end
    IntCalNum = CalIds.CalibrationIdIc(1);
	ExtCalNum = CalIds.CalibrationIdEc(1);
 

end

