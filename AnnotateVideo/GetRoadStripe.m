%   Make a stripe.
    NRoadDots = StripeNDots*6+3;
    RoadStripe = table('Size', [NRoadDots 3], 'VariableTypes', ["double","double","double"], 'VariableNames', ["X", "Y", "Z"]);
    RoadStripe.Z(:) = -1.0;
    RoadStripe.Y(1:StripeNDots*2+1) = -StripeDy;
    RoadStripe.Y(StripeNDots*2+2:StripeNDots*4+2) = 0.0;
    RoadStripe.Y(StripeNDots*4+3:StripeNDots*6+3) = StripeDy;
    xtemp = -StripeDx*StripeNDots:StripeDx:StripeDx*StripeNDots;
    RoadStripe.X(1:StripeNDots*2+1) = xtemp';
    RoadStripe.X(StripeNDots*2+2:StripeNDots*4+2) = xtemp';
    RoadStripe.X(StripeNDots*4+3:StripeNDots*6+3) = xtemp';
    clear xtemp;

%   Cast points into the camera's axis system.    
    [RoadStripe.XTC, RoadStripe.YTC, RoadStripe.ZTC] = ...
        CastVehicleCoordstoCamera(RoadStripe.X, RoadStripe.Y, RoadStripe.Z, CameraExtCal);    
%   Eliminate any point that is behind the camera or well outside of
%   the field of view.
    IsForward = (RoadStripe.ZTC > 0 & ...
                RoadStripe.XTC./RoadStripe.ZTC >= xpcmin & ...
                RoadStripe.XTC./RoadStripe.ZTC <= xpcmax & ...
                RoadStripe.YTC./RoadStripe.ZTC >= ypcmin & ...
                RoadStripe.YTC./RoadStripe.ZTC <= ypcmax);
    RoadStripe = RoadStripe(IsForward,:);
    
%   Get the row and column for each point detection.    
    [RoadStripe.col, RoadStripe.row] = ...
        ReverseCameraIntCal(CameraIntCal, RoadStripe.XTC, RoadStripe.YTC, RoadStripe.ZTC);  
