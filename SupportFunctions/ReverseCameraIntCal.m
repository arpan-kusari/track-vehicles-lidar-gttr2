function [col, row] = ReverseCameraIntCal(CameraIntCal, XC, YC, ZC)

X = XC./ZC;
Y = YC./ZC;
R2 = X.*X+Y.*Y;
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

RD = 1 + k1*R2 + k2*R2.*R2 + k3*R2.*R2.*R2;
XD = X.*RD + 2*p1*X.*Y + p2*(R2+2*X.*X);
YD = Y.*RD + 2*p2*X.*Y + p1*(R2+2*Y.*Y);

row = YD*fy + r0;
col = XD*fx + c0 + YD*skew;

row = round(row);
col = round(col);

end
