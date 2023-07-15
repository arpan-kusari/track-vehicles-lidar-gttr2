%       Get the radar calibration ID. (THIS MAY REQUIRE IMPROVEMENT LATER.)        
        [RadarCalNum] = GetRadarCalNumsbyRunId(RunId);
%       Find out which camera/radar pairs overlap.
        [FOVOverlap] = FOVOverlapTable(RadarCalNum, IntCalNum, ExtCalNum);  
        
        
%       Build the query.
        query = (' ');
        nRad = -1;
        nR = 0
        while (nRad < nRadars)
            nRad = nRad + 1;
            if (FOVOverlap.Overlap(FOVOverlap.CameraNum == CameraNum & FOVOverlap.RadarNum == nRad) == 1)
                if (nRad == 0)
                    nR = nR + 1;
%                   Get the radar detections from the ESR.
                    query = [query 'select TimeCs, XTV, YTV, Target, 0 as Radar, TargetStatus from DataEsrTargetsCalibratedCs'];
                    query = [query ' where RunId = ' num2str(RunId)];
                    query = [query, ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
                    if (RadarNoCoast == 1)
                       query = [query, ' and TargetStatus != 7 and TargetStatus != 4 and TargetStatus != 6'];
                    end
                else
%                   Get the radar detections from the GPR.
                    nR = nR + 1;
                    if (nR > 1)
                        query = [query ' union '];
                    end
                    query = ['select TimeCs, XTV, YTV, Target, ' num2str(nRad)]; 
                    query = [query ' as Radar, (2*(HistObj + 1) - MeasObj) as TargetStatus'];
                    query = [query ' from DataOhw' num2str(nRad) 'TargetsCalibratedCs '];
                    query = [query ' where RunId = ' num2str(RunId)];
                    query = [query, ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
                    if (RadarNoCoast == 1)
                        query = [query, ' and MeasObj = 1'];
                    end
                end
            end
        end
        query = [query 'order by Radar, Target, TimeCs'];
        RadarDetections = fetch(conn,query);
        RadarDetections.ZTV = 0.0*RadarDetections.XTV; 

        
%       Cast radar detections into the camera's axis system.    
        [RadarDetections.XTC, RadarDetections.YTC, RadarDetections.ZTC] = ...
            CastVehicleCoordstoCamera(RadarDetections.XTV, RadarDetections.YTV, RadarDetections.ZTV, CameraExtCal);    
%       Eliminate any detection that is behind the camera or well outside of
%       the field of view.
        IsForward = (RadarDetections.ZTC > 0 & ...
                    RadarDetections.XTC./RadarDetections.ZTC >= xpcmin & ...
                    RadarDetections.XTC./RadarDetections.ZTC <= xpcmax & ...
                    RadarDetections.YTC./RadarDetections.ZTC >= ypcmin & ...
                    RadarDetections.YTC./RadarDetections.ZTC <= ypcmax);
        RadarDetections = RadarDetections(IsForward,:);
    
%       Get the row and column for each POV detection.    
        [RadarDetections.col, RadarDetections.row] = ...
            ReverseCameraIntCal(CameraIntCal, RadarDetections.XTC, RadarDetections.YTC, RadarDetections.ZTC);

