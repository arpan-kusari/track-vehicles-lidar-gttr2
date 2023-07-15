%   Get Bounding boxes.
    query = ('select BRX, BRY, TLX, TLY, V51Frame, TimeCs, ObjConf, IndexId from ObjectsBasler');
    query = [query num2str(CameraNum) 'Cs'];
    query = [query, ' where RunId = ' num2str(RunId)];
    query = [query, ' and TimeCs <= ' num2str(TimeCsMax) ' and TimeCs >= ' num2str(TimeCsMin)];
    query = strcat(query, ' order by V51Frame');
    BBoxes = fetch(conn, query);

%   Cast bounding box coordinates into pixels.
	BBoxes.BRCol = round(ImageColumns*BBoxes.BRX);
	BBoxes.BRRow = round(ImageRows*BBoxes.BRY);
	BBoxes.TLCol = round(ImageColumns*BBoxes.TLX);
	BBoxes.TLRow = round(ImageRows*BBoxes.TLY);
