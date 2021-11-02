function cost = estimate_3D_GIOU(track,detection)
% ESTIMATE_3D_GIOU Utilize Generalized Intersection of Union to estimate 
% the cost between a current track and current detection
% both tracks and detections are 3D bounding boxes
% Algorithm from "3D-GIoU: 3D Generalized Intersection over Union for
% Object Detection in Point Cloud"
% rotate it back to axis-aligned coordinates
% Rot_mat = [cos(-track.Rot) -sin(-track.Rot); sin(-track.Rot) cos(-track.Rot)];
Rot_mat = [1 0; 0 1];
% get the bounding box as (x_min, y_min, x_max, y_max, rot)
[track_X, track_Y, track_Z, track_Sx, track_Sy, track_Sz, track_Rot] = matsplit(track.state(1:7));
bb_track_2D = [track_X - track_Sx/2,...
               track_Y - track_Sy/2,...
               track_X + track_Sx/2,...
               track_Y + track_Sy/2,...
               track_Rot];
% get the box coords to transform the box 
box_coords = [bb_track_2D(1) bb_track_2D(2);
              bb_track_2D(3) bb_track_2D(2);
              bb_track_2D(3) bb_track_2D(4);
              bb_track_2D(1) bb_track_2D(4)];

% transform to axis aligned box for easy calculation
box_coords_track_aligned = zeros(4, 2);
for i = 1:size(box_coords, 1)
    box_coords_track_aligned(i, :) = (Rot_mat*box_coords(i,:)')';
end
% bounding box of track aligned along axis
bb_track_aligned = [min(box_coords_track_aligned(:, 1)),...
                    min(box_coords_track_aligned(:, 2)),...
                    max(box_coords_track_aligned(:, 1)),...
                    max(box_coords_track_aligned(:, 2))];
bb_detection_2D = [detection.XV - detection.Sx/2,...
                          detection.YV - detection.Sy/2,...
                          detection.XV + detection.Sx/2,...
                          detection.YV + detection.Sy/2,...
                          detection.Rot];
% Rot_mat = [cos(-detection.Rot) -sin(-detection.Rot); sin(-detection.Rot) cos(-detection.Rot)];
Rot_mat = [1 0; 0 1];
box_coords = [bb_detection_2D(1) bb_detection_2D(2);
              bb_detection_2D(3) bb_detection_2D(2);
              bb_detection_2D(3) bb_detection_2D(4);
              bb_detection_2D(1) bb_detection_2D(4)];
% transform to axis aligned box for easy calculation
box_coords_detections_aligned = zeros(4, 2);
for i = 1:size(box_coords, 1)
    box_coords_detections_aligned(i, :) = (Rot_mat*box_coords(i,:)')';
end
bb_detections_aligned = [min(box_coords_detections_aligned(:, 1)),...
                         min(box_coords_detections_aligned(:, 2)),...
                         max(box_coords_detections_aligned(:, 1)),...
                         max(box_coords_detections_aligned(:, 2))];
boxes_coords = [bb_track_aligned(1) bb_track_aligned(2);
                bb_track_aligned(3) bb_track_aligned(2);
                bb_track_aligned(3) bb_track_aligned(4);
                bb_track_aligned(1) bb_track_aligned(4);
                bb_detections_aligned(1) bb_detections_aligned(2);
                bb_detections_aligned(3) bb_detections_aligned(2);
                bb_detections_aligned(3) bb_detections_aligned(4);
                bb_detections_aligned(1) bb_detections_aligned(4)];
bb_enclosed_2D = minBoundingBox(boxes_coords')';
area_track_2D = track_Sx*track_Sy;
area_detection_2D = detection.Sx*detection.Sy;
area_enclosed_2D = (max(bb_enclosed_2D(:, 1)) - min(bb_enclosed_2D(:, 1)))*(max(bb_enclosed_2D(:, 2)) - min(bb_enclosed_2D(:, 2)));
height_enclosed = max(track_Z + track_Sz, detection.ZV + detection.Sz) - min(track_Z - track_Sz, detection.ZV - detection.Sz);
bb_overlap_2D = [max([bb_track_2D(1), bb_detection_2D(1)]),...
                 max([bb_track_2D(2), bb_detection_2D(2)]),...
                 min([bb_track_2D(3), bb_detection_2D(3)]),...
                 min([bb_track_2D(4), bb_detection_2D(4)])];
area_overlap = (bb_overlap_2D(3) - bb_overlap_2D(1))*(bb_overlap_2D(4)-bb_overlap_2D(2));
height_overlap = min(track_Z + track_Sz, detection.ZV + detection.Sz) - max(track_Z - track_Sz, detection.ZV - detection.Sz);

if area_overlap <= 0
    intersection_3D = 0;
else
    if height_overlap <= 0
        intersection_3D = 0;
    else
        intersection_3D = area_overlap * height_overlap;
    end
end
vol_track = track_Sx * track_Sy * track_Sz;
vol_detection = detection.Sx * detection.Sy * detection.Sz;
vol_enclosed = area_enclosed_2D * height_enclosed;
union_vol = vol_track + vol_detection - intersection_3D;
IOU_3D = intersection_3D/union_vol;
GIOU_3D = IOU_3D - ((vol_enclosed - union_vol)/vol_enclosed);
cost = 1 - GIOU_3D;
% figure(1);
% plot([bb_enclosed_2D(:, 1); bb_enclosed_2D(1,1)], [bb_enclosed_2D(:, 2); bb_enclosed_2D(1,2)], 'k-');
% hold on;
% plot(boxes_coords(1:4, 1), boxes_coords(1:4, 2), 'r.'); plot(boxes_coords(5:8,1), boxes_coords(5:8, 2), 'b.');plot(bb_enclosed_2D(:, 1), bb_enclosed_2D(:, 2), 'k.');

end

