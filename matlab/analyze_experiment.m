function M = analyze_experiment(csvfile, t0)
%ANALYZE_EXPERIMENT Metrics of an experimental run with the same definitions used
%   in the manuscript. CSVFILE columns (header row allowed):
%     t [s], d [m], theta1 [rad], theta2 [rad], d_r, theta1_r, theta2_r,
%     u_d [N], u_1 [N m], u_2 [N m]
%   Link lengths must match the physical robot (edit L1, L2 below).
%   Returns joint RMSE, Cartesian RMSE/max error (mm) and total variation of the
%   command per second (TV). Example:  M = analyze_experiment('exp_S2_FOSMC.csv', 1.0)
if nargin < 2, t0 = 1.0; end
L1 = 0.325; L2 = 0.325;
D = readmatrix(csvfile);
t = D(:,1); q = D(:,2:4); qr = D(:,5:7); u = D(:,8:10);
fk = @(q) [q(:,1) + L1*cos(q(:,2)) + L2*cos(q(:,2)+q(:,3)), L1*sin(q(:,2)) + L2*sin(q(:,2)+q(:,3))];
m = t >= t0;
ex = sqrt(sum((fk(q) - fk(qr)).^2, 2)) * 1e3;
M.joint_rmse = sqrt(mean((q(m,:) - qr(m,:)).^2, 1));
M.cart_rmse_mm = sqrt(mean(ex(m).^2));
M.cart_max_mm = max(ex(m));
M.TV = sum(sum(abs(diff(u(m,:))), 1)) / (t(find(m, 1, 'last')) - t(find(m, 1)));
fprintf('Cartesian RMSE %.4f mm | max %.4f mm | TV %.1f\n', M.cart_rmse_mm, M.cart_max_mm, M.TV);
end
