function [rotated] = Body321(unrotated,roll, pitch, yaw)

    d2r = 3.14159265358979/180.0;
    c1 = cos(roll*d2r);
    c2 = cos(pitch*d2r);
    c3 = cos(yaw*d2r);
    s1 = sin(roll*d2r);
    s2 = sin(pitch*d2r);
    s3 = sin(yaw*d2r);

	R(1, 1) = c2*c3;
    R(1, 2) = c2*s3;
    R(1, 3) = -s2;
    R(2, 1) = (s1*s2*c3 - c1*s3);
    R(2, 2) = (s1*s2*s3 + c1*c3);
    R(2, 3) = s1*c2;
    R(3, 1) = (c1*s2*c3 + s1*s3);
    R(3, 2) = (c1*s2*s3 - s1*c3);
    R(3, 3) = c1*c2;
%R = angle2dcm(yaw*d2r, pitch*d2r, roll*d2r);
rotated = R*unrotated;

end

