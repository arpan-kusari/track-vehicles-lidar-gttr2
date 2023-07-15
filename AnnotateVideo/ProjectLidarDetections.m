%   Identify lidar detections that occur near the time of this frame.
    TheseLidarDetections = (abs(LidarDetections.TimeCs - TimeCs) <= LidarTimeCsTol & LidarDetections.ZTC < LidarRangeThresh);
    LidarDetectionsSubset = LidarDetections(TheseLidarDetections,:);
    nLDet = height(LidarDetectionsSubset);
    
%   Project each one.
	nld = 0;
	while (nld < nLDet)
        nld = nld + 1;
        col = LidarDetectionsSubset.col(nld);
        row = LidarDetectionsSubset.row(nld);
        ZC = LidarDetectionsSubset.ZTC(nld);
        nObjCnt = LidarDetectionsSubset.ObjCnt(nld);
        nL2 = LidarDetectionsSubset.Lidar(nld);
        MarkerDiameter = max(1, PixelsatMax*(1 - ZC/MarkerRangeMax));
        if (col > 0 && col <=1920 && row > 0 && row <= 1200)
            img = insertShape(img, 'Circle',[col row MarkerDiameter], 'Color', 'green', 'LineWidth', 6);
            note = strcat(num2str(nL2),'-',num2str(nObjCnt));
            img = insertText(img, [col row+round(MarkerDiameter/2)], note, 'TextColor', 'green', 'FontSize', 24, 'BoxOpacity', 0, 'AnchorPoint', 'LeftBottom');
        end
    end


    
%   Project bounding boxes (if asked).
%   THE CODE HERE IS INEFFICIENT.
	if (UseLidarBoundingBoxes == 1)

%       Get the subset for this TimeCs.        
        LDVertexColsSubset = LDVertexCols(TheseLidarDetections,:);
        LDVertexRowsSubset = LDVertexRows(TheseLidarDetections,:);

%       Project each one.
        nld = 0;
        while (nld < nLDet)
            nld = nld + 1;
            col1 = LDVertexColsSubset(nld, 1); row1 = LDVertexRowsSubset(nld, 1);
            col2 = LDVertexColsSubset(nld, 2); row2 = LDVertexRowsSubset(nld, 2);
            col3 = LDVertexColsSubset(nld, 3); row3 = LDVertexRowsSubset(nld, 3);
            col4 = LDVertexColsSubset(nld, 4); row4 = LDVertexRowsSubset(nld, 4);
            col5 = LDVertexColsSubset(nld, 5); row5 = LDVertexRowsSubset(nld, 5);
            col6 = LDVertexColsSubset(nld, 6); row6 = LDVertexRowsSubset(nld, 6);
            col7 = LDVertexColsSubset(nld, 7); row7 = LDVertexRowsSubset(nld, 7);
            col8 = LDVertexColsSubset(nld, 8); row8 = LDVertexRowsSubset(nld, 8);

            img = insertShape(img, 'Line',[col1 row1 col2 row2], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col1 row1 col3 row3], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col2 row2 col4 row4], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col3 row3 col4 row4], 'Color', 'green', 'LineWidth', 2);

            img = insertShape(img, 'Line',[col5 row5 col6 row6], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col5 row5 col7 row7], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col6 row6 col8 row8], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col7 row7 col8 row8], 'Color', 'green', 'LineWidth', 2);

            img = insertShape(img, 'Line',[col1 row1 col5 row5], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col2 row2 col6 row6], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col3 row3 col7 row7], 'Color', 'green', 'LineWidth', 2);
            img = insertShape(img, 'Line',[col4 row4 col8 row8], 'Color', 'green', 'LineWidth', 2);
        end
    end
    