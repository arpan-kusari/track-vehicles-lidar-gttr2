%   Project dots for a mock road stripe.
	nrows = size(RoadStripe, 1);
	n = 1;
	while (n <=nrows)
        row = RoadStripe.row(n);
        col = RoadStripe.col(n);
        if (row > 0 && row <= 1200 && col > 0 && col <= 1920)
             img = insertShape(img, 'FilledCircle',[col row 6], 'Color', 'white', 'LineWidth', 3, 'opacity', 1);
        end
        n = n + 1;
    end
