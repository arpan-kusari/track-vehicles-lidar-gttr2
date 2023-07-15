function [Overlap] = DoFOVsOverlap(RadarNum, RadarCalibrationId, CameraNum, CameraIntCal, CameraExtCalId)
%      This function returns 1 if overlap between the camera FOV and radar
%      FOV is possible.
%       The algorthm is not all that smart - It checks three points far
%       away from the radar sensor at the center and edges of the field of
%       view to determine if any of them are within the camera field of
%       view.

    D2R = 3.14159265358979323846264338327950288419716939937510/180.;
    Overlap = 0;
    
%   Determine the horizontal field of view of the camera. 
	[xpcmin, xpcmax, ~, ~] = GetProjectionCoordLimits(CameraIntCal);

%   Establish communication with the databases.
    conn = database('GTTR4CPU',' ',' ');

%   Get radar calibration parameters.
    query = ('select YawRadar, XRadar, YRadar from RadarExtrinsicCals ');
    query = [query 'where CalibrationId = ' num2str(RadarCalibrationId)];
    query = [query ' and Radar = ' num2str(RadarNum)];
    RadarExtCal = fetch(conn,query); 

%   Get camera extrinsic calibration parameters.
    fields = (' XCamera, YCamera, ZCamera, RollCamera, PitchCamera, YawCamera ');
    query = ['select ' fields ' from CameraExtrinsicCals '];
    query = [query 'where CalibrationId = ' num2str(CameraExtCalId)];
    query = [query ' and Camera = ' num2str(CameraNum)];
    CameraExtCal = fetch(conn,query);


%   Try the three points.
    if (RadarNum == 0)
        RadarFOV = 51.2;
    else
        RadarFOV = 45.; %   PLACEHOLDER. THIS SHOULD BE PART OF THE CAL TABLE.
    end
    Range = 120.;   %   Set this to "far".
    ZR = 0.0;       %   Assume a Z coordinate for radar detections.
    n = -1;
    while (n < 2)
%       Azimuth.
        cra = cos(D2R*n*RadarFOV);
        sra = sin(D2R*n*RadarFOV);
%       Radar sensor yaw.
        cr3 = cos(D2R*RadarExtCal.YawRadar(1));
        sr3 = sin(D2R*RadarExtCal.YawRadar(1));
        
%       Get reference point location relative to the camera, but in the
%       vehicle axis system.
        ur(1,1) = RadarExtCal.XRadar(1) + Range*(cr3*cra - sr3*sra) - CameraExtCal.XCamera(1);
        ur(2,1) = RadarExtCal.YRadar(1) + Range*(sr3*cra + cr3*sra) - CameraExtCal.YCamera(1);
        ur(3,1) = ZR - CameraExtCal.ZCamera(1);
        
%       Rotate this to the camera axis system.
        r = Body321(ur, CameraExtCal.RollCamera(1), CameraExtCal.PitchCamera(1), CameraExtCal.YawCamera(1));
     
%       If this point is within the camera's azimuth, return 1.
        if (r(3) > 0 && r(1)/r(3) <= xpcmax && r(1)/r(3) >= xpcmin)
            Overlap = 1;
            return;
        end
        
        
        n = n + 1;
    end
end

