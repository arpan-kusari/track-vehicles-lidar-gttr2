function [xpcmin, xpcmax, ypcmin, ypcmax] = GetProjectionCoordLimits(CameraIntCals)
%      This function derives the projection coordinate limits of a video
%      frame using intrinsic calibration parameters.

%   Perform the calculation for each camera intrinsic calibration.
    nic = height(CameraIntCals);
    xpcmin = zeros(nic, 1); xpcmax = zeros(nic, 1); 
    ypcmin = zeros(nic, 1); ypcmax = zeros(nic, 1); 
    n = 0;
    while (n <nic)
        n = n + 1;
        
%       Left edge.
        [xpctopleft, ypctopleft] = ApplyIntrinsicCal(0, 0, CameraIntCals(n,:));
        [xpccenterleft, ~] = ApplyIntrinsicCal(0, 0.5, CameraIntCals(n,:));
        [xpcbottomleft, ypcbottomleft] = ApplyIntrinsicCal(0, 1, CameraIntCals(n,:));

%       Right edge.
        [xpctopright, ypctopright] = ApplyIntrinsicCal(1, 0, CameraIntCals(n,:));
        [xpccenterright, ~] = ApplyIntrinsicCal(1, 0.5, CameraIntCals(n,:));
        [xpcbottomright, ypcbottomright] = ApplyIntrinsicCal(1, 1, CameraIntCals(n,:));

%       Top and bottom edge.
        [~, ypctopcenter] = ApplyIntrinsicCal(0.5, 0, CameraIntCals(n,:));
        [~, ypcbottomcenter] = ApplyIntrinsicCal(0.5, 1, CameraIntCals(n,:));

%       Grab limits.    
        xpcmin(n,1) = min([xpctopleft xpccenterleft xpcbottomleft]);
        xpcmax(n,1) = max([xpctopright xpccenterright xpcbottomright]);
        ypcmin(n,1) = min([ypctopleft ypctopcenter ypctopright]);
        ypcmax(n,1) = max([ypcbottomleft ypcbottomcenter ypcbottomright]);
    end
    
end
