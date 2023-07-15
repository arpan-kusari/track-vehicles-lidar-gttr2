function [XTC, YTC, ZTC] = CastVehicleCoordstoCamera(XTV, YTV, ZTV, CameraExtCal)
%	This function casts detections in vehicle coordinates (XTV, YTV, ZTV) 
%   into camera coordinates (XTC, YTC, ZTC).
%   The camera number and calibration ID are needed to get calibration
%   parameters.
%

%   Shift the detections to the camera origin.    
    PTI(1, :) = XTV - CameraExtCal.XCamera;
    PTI(2, :) = YTV - CameraExtCal.YCamera;
    PTI(3, :) = ZTV - CameraExtCal.ZCamera;
    
%   Rotate the detections to the camera axis system.    
    PTC = Body321(PTI, CameraExtCal.RollCamera, CameraExtCal.PitchCamera, CameraExtCal.YawCamera)';
    XTC = PTC(:, 1);
    YTC = PTC(:, 2);
    ZTC = PTC(:, 3);    
