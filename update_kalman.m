function [x_est,cov_est] = update_kalman(x_new,cov_new, meas, meas_noise_mat)
%UPDATE_KALMAN Update is correction based on new measurement
%   Detailed explanation goes here
H = [eye(7, 7) zeros(7, 7)];
R = meas_noise_mat;
y = meas - H*x_new;
S = H*cov_new*H' + R;
K = (cov_new*H')/S;
x_est = x_new + K*y;
cov_est = (eye(14, 14) - K*H)*cov_new;
end

