%  Identify radar detections that occur near the time of this frame.
    TheseRadarDetections = (abs(RadarEvents.TimeCs - TimeCs) < outfile.FrameRate & RadarEvents.ZTC < RadarRangeThresh);
    RadarEventsSubset = RadarEvents(TheseRadarDetections,:);
 
%   Get a list of the surviving event numbers.
    EventSubset = unique(RadarEventsSubset.EventId);
	nREv = size(EventSubset, 1);
%   For each event.......
	nre = 0;
	while (nre < nREv)
        nre = nre + 1;
 %      Get the surviving detections for this event.
        ThisEvent = (EventSubset(nre) == RadarEventsSubset.EventId);
        RadarSubSub = RadarEventsSubset(ThisEvent,:);
        nRSS = height(RadarSubSub);
 %      If interpolation to TimeCs is possible, do so.
        if (nRSS > 1)
            col = round(interp1(RadarSubSub.TimeCs, RadarSubSub.col, TimeCs,'linear',-1));
            row = round(interp1(RadarSubSub.TimeCs, RadarSubSub.row, TimeCs,'linear',-1));
            ZC = round(interp1(RadarSubSub.TimeCs, RadarSubSub.ZTC, TimeCs,'linear',-1));
            nEvent = EventSubset(nre);
            nR2 = RadarSubSub.Radar(1);
            MarkerDiameter = max(1, PixelsatMax*(1 - ZC/MarkerRangeMax));
            if (col > 0 && col <=1920 && row > 0 && row <= 1200)
                img = insertShape(img, 'Circle',[col row MarkerDiameter], 'Color', 'red', 'LineWidth', 6);
                note = strcat(num2str(nR2),'-',num2str(nEvent));
                img = insertText(img, [col row+round(MarkerDiameter/2)-0*5*mod(nEvent,10)], note, ...
                    'TextColor', 'red', 'FontSize', 24, 'BoxOpacity', 0, 'AnchorPoint', 'LeftBottom');
            end
        end
	end
