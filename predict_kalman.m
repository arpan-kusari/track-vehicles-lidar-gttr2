function [x_new,cov_new] = predict_kalman(x,cov, dt)
% PREDICT_KALMAN It predicts the Kalman filter output from a track and a detection
% pair
% We run with the constant velocity model 
F = eye(14, 14);
F(1:7, 8:14) = eye(7, 7)*dt;
Q = eye(14, 14)*0.01;
x_new = F*x;
cov_new = F*cov*F' + Q;
end

