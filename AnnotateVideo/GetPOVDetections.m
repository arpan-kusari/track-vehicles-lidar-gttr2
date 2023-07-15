%       Get POV detections.
        query = ('select * from PovTargetLocations');
        query = [query, ' where RunId = ' num2str(RunId)];
        query = [query, ' and TimeCs % 10 = 0'];
        query = [query, ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
        query = [query, ' order by TimeCs'];
        POVDetections = fetch(conns, query);

%       Cast POV detections into the camera's axis system.    
        [POVDetections.XTC, POVDetections.YTC, POVDetections.ZTC] = ...
            CastVehicleCoordstoCamera(POVDetections.XTV, POVDetections.YTV, POVDetections.ZTV, CameraExtCal);    
%       Eliminate any detection that is behind the camera or well outside of
%       the field of view.
        IsForward = (POVDetections.ZTC > 0 & ...
                    POVDetections.XTC./POVDetections.ZTC >= xpcmin & ...
                    POVDetections.XTC./POVDetections.ZTC <= xpcmax & ...
                    POVDetections.YTC./POVDetections.ZTC >= ypcmin & ...
                    POVDetections.YTC./POVDetections.ZTC <= ypcmax);
        POVDetections = POVDetections(IsForward, :);
    
%       Get the row and column for each POV detection.    
        [POVDetections.col, POVDetections.row] = ...
            ReverseCameraIntCal(CameraIntCal, POVDetections.XTC, POVDetections.YTC, POVDetections.ZTC);  
