% Calculate the radius of the earth based on latitude.
function [REarth] = GetEarthRadius(Latitude)

	D2R = 3.1415926/180.;
	REquator = 6378137.;
	RPoles = 6356752.;
	c = cos(Latitude*D2R);
	s = sin(Latitude*D2R);
	REarth = sqrt(    ((c*REquator^2)^2 + (s*RPoles^2)^2)/((c*REquator)^2 + (s*RPoles)^2));
end
