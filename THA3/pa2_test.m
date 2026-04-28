%% pa2_test.m
% Test suite for PA2: Eye-in-Hand (AX=XB) calibration.
% Run from the THA3/ directory.
%
% Sections:
%   1. Clean data (all 10 configurations)
%   2. Noisy data (all 10 configurations)
%   3. Noisy data (first 5 configurations only)
%
% Each section solves for X and reports R_x, P_x, and verification metrics.

clear; clc;

% Add THA1 and THA2 for shared helpers (quat2rotm_my, vecToSO3, etc.)
addpath(fullfile(fileparts(mfilename('fullpath')), '../THA1'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../THA2'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'HW3-PA2'));

%% =========================================================
%  SECTION 1: Clean Data — All 10 Configurations
% =========================================================
fprintf('========================================\n');
fprintf(' SECTION 1: Clean Data (10 configs)\n');
fprintf('========================================\n');

[q_E, q_S, t_E, t_S] = data_quaternion();
[T_clean, res_clean] = eyeinhand_cal(q_E, q_S, t_E, t_S);

print_result('Clean (N=10)', T_clean, res_clean);
AXB_err_clean = verify_AXB(q_E, q_S, t_E, t_S, T_clean);

%% =========================================================
%  SECTION 2: Noisy Data — All 10 Configurations
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 2: Noisy Data (10 configs)\n');
fprintf('========================================\n');

[q_E_n, q_S_n, t_E_n, t_S_n] = data_quaternion_noisy();
[T_noisy, res_noisy] = eyeinhand_cal(q_E_n, q_S_n, t_E_n, t_S_n);

print_result('Noisy (N=10)', T_noisy, res_noisy);
AXB_err_noisy = verify_AXB(q_E_n, q_S_n, t_E_n, t_S_n, T_noisy);

%% =========================================================
%  SECTION 3: Noisy Data — First 5 Configurations
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 3: Noisy Data (5 configs)\n');
fprintf('========================================\n');

idx5 = 1:5;
[T_half, res_half] = eyeinhand_cal(q_E_n(idx5,:), q_S_n(idx5,:), ...
                                    t_E_n(idx5,:), t_S_n(idx5,:));

print_result('Noisy (N=5)', T_half, res_half);
AXB_err_half = verify_AXB(q_E_n(idx5,:), q_S_n(idx5,:), ...
                           t_E_n(idx5,:), t_S_n(idx5,:), T_half);

%% =========================================================
%  SUMMARY: Compare All Three
% =========================================================
fprintf('\n========================================\n');
fprintf(' SUMMARY\n');
fprintf('========================================\n');

fprintf('\n  Rotation difference (Frobenius norm of R_diff):\n');
R_clean = T_clean(1:3,1:3);
R_noisy = T_noisy(1:3,1:3);
R_half  = T_half(1:3,1:3);
fprintf('    Clean vs Noisy(10):  %.6f\n', norm(R_clean - R_noisy, 'fro'));
fprintf('    Clean vs Noisy(5):   %.6f\n', norm(R_clean - R_half, 'fro'));
fprintf('    Noisy(10) vs Noisy(5): %.6f\n', norm(R_noisy - R_half, 'fro'));

fprintf('\n  Translation difference (Euclidean norm of P_diff):\n');
P_clean = T_clean(1:3,4);
P_noisy = T_noisy(1:3,4);
P_half  = T_half(1:3,4);
fprintf('    Clean vs Noisy(10):  %.6f\n', norm(P_clean - P_noisy));
fprintf('    Clean vs Noisy(5):   %.6f\n', norm(P_clean - P_half));
fprintf('    Noisy(10) vs Noisy(5): %.6f\n', norm(P_noisy - P_half));

fprintf('\n  Mean AX=XB residual (should be ~0):\n');
fprintf('    Clean (N=10):  %.6f\n', AXB_err_clean);
fprintf('    Noisy (N=10):  %.6f\n', AXB_err_noisy);
fprintf('    Noisy (N=5):   %.6f\n', AXB_err_half);
fprintf('\n');

%% =========================================================
%  LOCAL HELPERS
% =========================================================

function print_result(label, T, res)
    R = T(1:3,1:3);
    P = T(1:3,4);
    fprintf('\n  --- %s ---\n', label);
    fprintf('  R_x:\n');
    fprintf('    [%8.5f %8.5f %8.5f]\n', R');
    fprintf('  P_x: [%.5f, %.5f, %.5f]\n', P);
    fprintf('  det(R_x): %.6f\n', det(R));
    fprintf('  lsqr flag=%d, relres=%.2e\n', res(1), res(2));
end

function mean_err = verify_AXB(q_E, q_S, t_E, t_S, X)
    % Check AX = XB for each consecutive pair.
    % A = E_i^{-1} * E_{i+1},  B = S_i * S_{i+1}^{-1}
    R_E = quat2rotm_my(q_E);
    R_S = quat2rotm_my(q_S);
    N = size(q_E, 1);
    errs = zeros(N-1, 1);
    for i = 1:N-1
        T_Ei  = [R_E(:,:,i)   t_E(i,:)';   0 0 0 1];
        T_Ei1 = [R_E(:,:,i+1) t_E(i+1,:)'; 0 0 0 1];
        T_Si  = [R_S(:,:,i)   t_S(i,:)';   0 0 0 1];
        T_Si1 = [R_S(:,:,i+1) t_S(i+1,:)'; 0 0 0 1];

        A = T_Ei \ T_Ei1;
        B = T_Si / T_Si1;

        AX = A * X;
        XB = X * B;
        errs(i) = norm(AX - XB, 'fro');
    end
    mean_err = mean(errs);
    fprintf('  AX=XB residuals per pair:\n');
    for i = 1:numel(errs)
        fprintf('    pair %d-%d: %.6f\n', i, i+1, errs(i));
    end
    fprintf('  Mean AX=XB residual: %.6f\n', mean_err);
end
