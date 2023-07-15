%   Identify radar detections that occur near the time of this frame.
    TheseRadarDetections = (abs(RadarDetections.TimeCs - TimeCs) <= RadarTimeCsTol & RadarDetections.ZTC < RadarRangeThresh);
    RadarDetectionsSubset = RadarDetections(TheseRadarDetections,:);
    nRDet = height(RadarDetectionsSubset);
    
%   Project each one.
	nrd = 0;
	while (nrd < nRDet)
        nrd = nrd + 1;
        col = RadarDetectionsSubset.col(nrd);
        row = RadarDetectionsSubset.row(nrd);
        ZC = RadarDetectionsSubset.ZTC(nrd);
        nTarget = RadarDetectionsSubset.Target(nrd);
        nR2 = RadarDetectionsSubset.Radar(nrd);
        MarkerDiameter = max(1, PixelsatMax*(1 - ZC/MarkerRangeMax));
        if (col > 0 && col <=1920 && row > 0 && row <= 1200)
            img = insertShape(img, 'Circle',[col row MarkerDiameter], 'Color', 'red', 'LineWidth', 6);
            note = strcat(num2str(nR2),'-',num2str(nTarget));
            img = insertText(img, [col row+round(MarkerDiameter/2)], note, 'TextColor', 'red', 'FontSize', 24, 'BoxOpacity', 0, 'AnchorPoint', 'LeftBottom');
        end
	end
    
   