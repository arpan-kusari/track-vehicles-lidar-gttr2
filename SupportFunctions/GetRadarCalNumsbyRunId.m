function [ExtCalNum] = GetRadarCalNumsbyRunId(RunId)
%   This function seeks out the radar extrinsic calibration ID using run number. 

%   Establish communication with the databases.
    conn = database('GTTR4CPU',' ',' ');

%   Query.
    query = ('select distinct CalibrationId');
    query = [query ' from DataESRTargetsCalibrated'];
    query = [query ' where RunId = ' num2str(RunId)];
    CalIds = fetch(conn, query);
    ncal = height(CalIds);
    if (ncal < 1)
        warning('Could not get video cal numbers.');
        return
    elseif (ncal > 1)
        warning('More than 1 set of video cal numbers.');        
    end
    if (isnan(CalIds.CalibrationId(1)))
        warning('Null extrinsic calibration ID.');        
        CalIds.CalibrationId(1) = 1;
    end
	ExtCalNum = CalIds.CalibrationId(1);
end

