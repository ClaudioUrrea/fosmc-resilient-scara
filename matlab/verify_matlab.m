%VERIFY_MATLAB Cross-implementation check against the Python reference trajectories.
%   Runs the deterministic verification cases (no noise; true plant = nominal with
%   friction x1.2 and a 1.5-kg payload) and compares them with results/verify_*.csv.
%   Run it from the matlab/ folder of the package (MATLAB or GNU Octave).
here = fileparts(mfilename('fullpath'));
cands = {fullfile(here, '..', 'results'), fullfile(here, 'results'), fullfile(pwd, 'results'), ...
         fullfile(pwd, '..', 'results'), here};
rdir = '';
for i = 1:numel(cands)
    if exist(fullfile(cands{i}, 'verify_S2_FOSMC.csv'), 'file'), rdir = cands{i}; break; end
end
if isempty(rdir)
    error(['Reference files verify_*.csv not found. Run this script from the matlab/ folder ', ...
           'of FOSMC_FractalFract (the CSVs are in ../results), or regenerate them with: python verify_py.py']);
end
fprintf('Reference folder: %s\n', rdir);
fprintf('fosmc_sim used:   %s\n', which('fosmc_sim'));
chk = fosmc_sim(struct('Tf', 0.002));
fprintf('Model link lengths: l1 = %.3f m, l2 = %.3f m\n', chk.params.l1, chk.params.l2);
if abs(chk.params.l1 - 0.400) > 1e-9 || abs(chk.params.l2 - 0.250) > 1e-9
    error(['An outdated fosmc_sim.m is being used (expected l1 = 0.400, l2 = 0.250). ', ...
           'Run verify_matlab from the matlab folder of the NEW package, or remove old copies from the MATLAB path.']);
end
cases = {'S2','FOSMC',0.7; 'S3','FOSMC',0.7; 'S2','ISMC',1.0; 'S3','ISMC',1.0};
fid = fopen(fullfile(here, 'verify_report.txt'), 'w');
for i = 1:size(cases, 1)
    sc = cases{i,1}; nm = cases{i,2}; lam = cases{i,3};
    o = fosmc_sim(struct('lam', lam, 'L2', 1600, 'scenario', sc));
    P = dlmread(fullfile(rdir, sprintf('verify_%s_%s.csv', sc, nm)), ',', 1, 0);
    dq = max(abs(P(:, 2:4) - o.q), [], 1);
    dx = max(sqrt(sum((P(:, 8:9) - o.x).^2, 2))) * 1e3;
    msg = sprintf('%s %-5s  max|dq| = [%.2e %.2e %.2e]  max|dx| = %.2e mm  RMSE(MATLAB) = %.4f mm  lock isolated at %.3f s\n', ...
        sc, nm, dq, dx, o.rmse, o.detected);
    fprintf('%s', msg); fprintf(fid, '%s', msg);
end
fclose(fid);
