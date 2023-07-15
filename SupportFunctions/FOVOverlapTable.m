function [FOVOverlap] = FOVOverlapTable(RadarCalibrationId, CameraIntCalId, CameraExtCalId)
%   This function creates a table that indicates which pairs of radar and
%   video have overlapping fields of view. 
%   The function assumes RadarNum ranges from 0-6, and CameraNum ranges
%   from 1-8.

%   Initialize the table.
    FOVOverlap = table('Size',[56 3],'VariableTypes',["double","double","double"],'VariableNames',["CameraNum","RadarNum","Overlap"]);
    
%   Loop through the cameras.
    m = 0;
    CameraNum = 1;
    while (CameraNum < 9)
%       Get the intrinsic calibration.
        [CameraIntCal] = GetIntrinsicCalParameters(CameraNum, CameraIntCalId);
%       Loop through the radars.
        RadarNum = 0;
        while (RadarNum < 7)
            m = m + 1;
            FOVOverlap.CameraNum(m) = CameraNum;
            FOVOverlap.RadarNum(m) = RadarNum;
            FOVOverlap.Overlap(m) = DoFOVsOverlap(RadarNum, RadarCalibrationId, CameraNum, CameraIntCal, CameraExtCalId);
            RadarNum = RadarNum + 1;
        end
        CameraNum = CameraNum + 1;
    end
    return
end

