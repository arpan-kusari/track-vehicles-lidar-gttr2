function [xic, yic] = ApplyIntrinsicCal(xin, yin, CameraIntCal)
%   This function applies the camera intrinsic calibration.
%   The input is image coordinates expressed as a fraction (0-1) of the image
%   from the top left corner.
%   The output is projection coordinates.

%[CameraIntCal] = GetIntrinsicCalParameters(CameraNum, CalibrationId);

%   Shuffle the parameters to compact variable names.
	k1 = CameraIntCal.RadialDistK1(1);
	k2 = CameraIntCal.RadialDistK2(1);
	k3 = CameraIntCal.RadialDistK3(1);
    p1 = CameraIntCal.TangentialDistP1(1);
    p2 = CameraIntCal.TangentialDistP2(1);
    c0 = CameraIntCal.PrincipalCol(1);
    r0 = CameraIntCal.PrincipalRow(1);
    skew = CameraIntCal.Skew(1);
    fx = CameraIntCal.FocalLengthX(1);
    fy = CameraIntCal.FocalLengthY(1);
    nrows = CameraIntCal.NumRows(1);
    ncols = CameraIntCal.NumCols(1);

%   Initialization and parameters.
    eps = 0.000001;
    maxiter = 100;

%   Initialize for iteration.    
    ydist = (yin*nrows - r0)/fy;
    xdist = (xin*ncols - c0 - ydist*skew)/fx;
    yg = ydist;
    xg = xdist;

%   Iterate to converge on xdist, ydist.    
    n = 0;
    while n < maxiter
    	n = n + 1;

%   	Function values.
		r2 = xg * xg + yg * yg;
		dum = (1 + r2 * (k1 + r2 * (k2 + k3 * r2)));
		F1 = xg * dum + 2 * p1*xg*yg + p2 * (r2 + 2 * xg*xg) - xdist;
		F2 = yg * dum + 2 * p2*xg*yg + p1 * (r2 + 2 * yg*yg) - ydist;
%       Jacobian values.
		dum2 = k1 + r2 * (2 * k2 + 3 * k3*r2);
		J11 = dum + 2 * xg*xg*dum2 + 2 * p1*yg + 6 * p2*xg;
		J22 = dum + 2 * yg*yg*dum2 + 6 * p1*yg + 2 * p2*xg;
		J12 = 2 * xg*yg*dum2 + 2 * p1*xg + 2 * p2*yg;

%       Refine the current guess.
		detJ = J11 * J22 - J12 * J12;
		xic = xg - (J22*F1 - J12 * F2) / detJ;
		yic = yg - (J11*F2 - J12 * F1) / detJ;

%       Check for convergence.
		if ((abs(xic - xg) < eps) && (abs(yic - yg) < eps))
			return;
        end
 
%       Set a new guess.
		xg = xic;
		yg = yic;
    end
