%       Get the radar calibration ID. (THIS MAY REQUIRE IMPROVEMENT LATER.)        
        [RadarCalNum] = GetRadarCalNumsbyRunId(RunId);
%       Find out which camera/radar pairs overlap.
        [FOVOverlap] = FOVOverlapTable(RadarCalNum, IntCalNum, ExtCalNum);  
        
        
%       Get a list of relevant radar events.
        query = ('select EventId, StartTimeCs, EndTimeCs, RadarId as Radar, DataCount, CalibrationId from DataRadarTargetEvents');
        queryadd = (' ');
        queryadd = [queryadd 'where RunId = ' num2str(RunId)];
        queryadd = [queryadd, ' and StartTimeCs <= ' num2str(TimeCsMax) ' and EndTimeCs >= ' num2str(TimeCsMin)];
%       Exclude radar units without intesecting FOV.        
        nRad = -1;
        while (nRad < nRadars)
            nRad = nRad + 1;
            if (FOVOverlap.Overlap(FOVOverlap.CameraNum == CameraNum & FOVOverlap.RadarNum == nRad) == 0)
                queryadd = [queryadd ' and RadarId <> ' num2str(nRad)];
            end
        end
        query = [query queryadd];
        RadarEventList = fetch(conna,query);

%       Get the radar detections.
        query = ('select TimeCs, XTV, YTV, EventId, RadarId as Radar, TargetStatus from DataRadarTargetTh');
        query = [query ' where RunId = ' num2str(RunId)];
        query = [query ' and EventId in (select distinct(EventId) from DataRadarTargetEvents' queryadd ')'];
        query = [query, ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
        if (RadarNoCoast == 1)
            query = [query ' and TargetStatus <= 3'];
        end
        query = [query ' order by RadarId, EventId, TimeCs'];
        RadarEvents = fetch(conna,query);
        RadarEvents.ZTV = 0.0*RadarEvents.XTV; 
        
%       FOR NOW!!!!
%        RadarEvents.TimeCs = round(RadarEvents.TimeCs, -1);

%       Cast radar detections into the camera's axis system.    
        [RadarEvents.XTC, RadarEvents.YTC, RadarEvents.ZTC] = ...
            CastVehicleCoordstoCamera(RadarEvents.XTV, RadarEvents.YTV, RadarEvents.ZTV, CameraExtCal);    
%       Eliminate any detection that is behind the camera or well outside of
%       the field of view.
        IsForward = (RadarEvents.ZTC > 0 & ...
                    RadarEvents.XTC./RadarEvents.ZTC >= xpcmin & ...
                    RadarEvents.XTC./RadarEvents.ZTC <= xpcmax & ...
                    RadarEvents.YTC./RadarEvents.ZTC >= ypcmin & ...
                    RadarEvents.YTC./RadarEvents.ZTC <= ypcmax);
        RadarEvents = RadarEvents(IsForward,:);
    
%       Get the row and column for each POV detection.    
        [RadarEvents.col, RadarEvents.row] = ...
            ReverseCameraIntCal(CameraIntCal, RadarEvents.XTC, RadarEvents.YTC, RadarEvents.ZTC);

