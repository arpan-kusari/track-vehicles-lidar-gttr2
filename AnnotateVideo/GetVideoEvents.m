%       Get the video detections from the events table.
        query = ('select Camera, EventNo, IndexId, V51Frame, TimeCs, BRXic, BRYic, TLXic, TLYic from BaslerTh');
        query = [query ' where RunId = ' num2str(RunId) ' and Camera = ' num2str(CameraNum)];    
        query = [query, ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
        query = [query ' order by Camera, EventNo, TimeCs'];
        BBoxesEvents = fetch(conna,query);
%       Get the rows and columns for each bounding box.    
        [BBoxesEvents.BRCol, BBoxesEvents.BRRow] = ...
            ReverseCameraIntCal(CameraIntCal, BBoxesEvents.BRXic, BBoxesEvents.BRYic, 1.);  
        [BBoxesEvents.TLCol, BBoxesEvents.TLRow] = ...
            ReverseCameraIntCal(CameraIntCal, BBoxesEvents.TLXic, BBoxesEvents.TLYic, 1.);  
