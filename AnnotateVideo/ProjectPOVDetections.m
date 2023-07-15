%   Project POV detections for the frame with this TimeCs.
    TheseRows = (POVDetections.TimeCs == TimeCs);
	POVSubset = POVDetections(TheseRows,:);
	nrows = size(POVSubset, 1);
	n = 1;
	while (n <=nrows)
        col = round(POVSubset.col(n));
        row = round(POVSubset.row(n));
        if (row > 0 && row <= 1200 && col > 0 && col <= 1920)
             img = insertShape(img, 'FilledCircle',[col row 10], 'Color', 'cyan', 'LineWidth', 3, 'opacity', 1);
        end
        n = n + 1;
	end
