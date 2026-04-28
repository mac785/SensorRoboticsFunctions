%% pa1_test.m
% Test suite for PA1 functions. Run from the THA3/ directory.
%
% Sections:
%   1. Data Ingestion    - verifies all readers load correct dimensions
%   2. C_expected        - verifies expected EM marker computation vs output1
%   3. Pivot Calibration - EM and optical pivot cal vs output1
%   4. Full PA1 Solve    - end-to-end pa1_solve + output file write
%
% Debug datasets a-g have known output1 files for validation.
% Unknown datasets h-k are run-only (no ground truth available).

clear; clc;

% Add THA2 so shared helpers (inv_transform, etc.) are found
% regardless of how/where MATLAB launched this script.
addpath(fullfile(fileparts(mfilename('fullpath')), '../THA2'));
addpath(fullfile(fileparts(mfilename('fullpath')), '../THA2/helpers'));

DATA_DIR  = 'HW3-PA1/';
OUT_DIR   = 'output/';
DEBUG_IDS = {'a','b','c','d','e','f','g'};

TOL_C   = 0.50;  % mm tolerance for C_expected per-component
TOL_PIV = 0.50;  % mm tolerance for pivot position (norm)

n_debug = numel(DEBUG_IDS);

% Accumulate per-dataset results for the summary table
results = struct();
for i = 1:n_debug
    id = DEBUG_IDS{i};
    results.(id).ingest_pass  = false;
    results.(id).C_pass       = false;
    results.(id).C_max_err    = NaN;
    results.(id).em_piv_pass  = false;
    results.(id).em_piv_err   = NaN;
    results.(id).opt_piv_pass = false;
    results.(id).opt_piv_err  = NaN;
    results.(id).solve_pass   = false;
    results.(id).solve_em_err = NaN;
    results.(id).solve_opt_err= NaN;
    results.(id).solve_C_err  = NaN;
end

%% =========================================================
%  SECTION 1: Data Ingestion
%  Checks that all five readers return consistent dimensions.
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 1: Data Ingestion\n');
fprintf('========================================\n');

for i = 1:n_debug
    id  = DEBUG_IDS{i};
    pfx = [DATA_DIR 'pa1-debug-' id '-'];

    try
        [N_body, d, a, c]                          = cal_body_data   ([pfx 'calbody.txt']);
        [N_read, frame_data]                        = cal_readings_data([pfx 'calreadings.txt']);
        [N_out,  P_em_ref, P_opt_ref, out_frames]  = output_1_data   ([pfx 'output1.txt']);
        [N_G, N_empiv_frames, G_frames]             = empivot_data    ([pfx 'empivot.txt']);
        [N_D, N_H, N_optpiv_frames, opt_frames]     = optpivot_data   ([pfx 'optpivot.txt']);

        % Dimension consistency checks
        assert(N_body(3) == N_read(3),  'N_c mismatch: calbody vs calreadings');
        assert(N_body(3) == N_out(1),   'N_c mismatch: calbody vs output1');
        assert(N_read(4) == N_out(2),   'N_frames mismatch: calreadings vs output1');
        assert(N_body(1) == N_D,        'N_D mismatch: calbody vs optpivot');
        assert(size(d,1) == N_body(1),  'Row count wrong for d');
        assert(size(a,1) == N_body(2),  'Row count wrong for a');
        assert(size(c,1) == N_body(3),  'Row count wrong for c');
        assert(numel(G_frames)   == N_empiv_frames,  'G_frames count wrong');
        assert(numel(opt_frames) == N_optpiv_frames, 'opt_frames count wrong');

        results.(id).ingest_pass = true;
        fprintf('  [PASS] dataset-%s  (N_c=%d, N_frames=%d, N_G=%d, N_H=%d)\n', ...
            id, N_body(3), N_read(4), N_G, N_H);
    catch ME
        fprintf('  [FAIL] dataset-%s  %s\n', id, ME.message);
    end
end

%% =========================================================
%  SECTION 2: C_expected Computation (Diagnostic)
%  Uses cloud2cloud to compute F_D and F_A each frame, then
%  computes C_expected = F_D^{-1} * F_A * c_i.
%  Reports deviation from output1 reference. Larger errors
%  for datasets with EM distortion/noise are expected.
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 2: C_expected (informational)\n');
fprintf('========================================\n');

for i = 1:n_debug
    id  = DEBUG_IDS{i};
    pfx = [DATA_DIR 'pa1-debug-' id '-'];

    if ~results.(id).ingest_pass
        fprintf('  [SKIP] dataset-%s  (ingestion failed)\n', id);
        continue;
    end

    try
        [N_body, d, a, c]      = cal_body_data   ([pfx 'calbody.txt']);
        [N_read, frame_data]   = cal_readings_data([pfx 'calreadings.txt']);
        [~, ~, ~, out_frames]  = output_1_data   ([pfx 'output1.txt']);

        N_frames = N_read(4);
        N_c      = N_body(3);

        max_err = 0;
        for k = 1:N_frames
            F_D     = cloud2cloud(d, frame_data{k}.D);
            F_A     = cloud2cloud(a, frame_data{k}.A);
            F_D_inv = inv_transform(F_D);

            C_exp = zeros(N_c, 3);
            for j = 1:N_c
                pt = F_D_inv * (F_A * [c(j,:)'; 1]);
                C_exp(j,:) = pt(1:3)';
            end

            err = abs(C_exp - out_frames{k}.C);
            max_err = max(max_err, max(err(:)));
        end

        results.(id).C_max_err = max_err;
        fprintf('  [INFO] dataset-%s  max_err=%.4f mm\n', id, max_err);
    catch ME
        fprintf('  [FAIL] dataset-%s  %s\n', id, ME.message);
    end
end

%% =========================================================
%  SECTION 3: Pivot Calibration
%  Tests pivot_cal directly for both EM and optical probes.
%  Optical H markers are transformed to EM frame before pivot_cal.
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 3: Pivot Calibration\n');
fprintf('========================================\n');

for i = 1:n_debug
    id  = DEBUG_IDS{i};
    pfx = [DATA_DIR 'pa1-debug-' id '-'];

    if ~results.(id).ingest_pass
        fprintf('  [SKIP] dataset-%s  (ingestion failed)\n', id);
        continue;
    end

    try
        [N_body, d, ~, ~]              = cal_body_data([pfx 'calbody.txt']);
        [~, P_em_ref, P_opt_ref, ~]    = output_1_data([pfx 'output1.txt']);
        [~, ~, G_frames]               = empivot_data ([pfx 'empivot.txt']);
        [~, ~, ~, opt_frames]          = optpivot_data([pfx 'optpivot.txt']);

        % --- EM pivot ---
        [P_em_col, ~] = pivot_cal(G_frames);
        em_err = norm(P_em_col' - P_em_ref);

        % --- Optical pivot: transform H to EM frame via inv(F_D) ---
        N_optframes = numel(opt_frames);
        H_em_frames = cell(N_optframes, 1);
        for k = 1:N_optframes
            F_D     = cloud2cloud(d, opt_frames{k}.D);
            F_D_inv = inv_transform(F_D);
            H       = opt_frames{k}.H;
            H_em    = zeros(size(H));
            for j = 1:size(H, 1)
                pt = F_D_inv * [H(j,:)'; 1];
                H_em(j,:) = pt(1:3)';
            end
            H_em_frames{k} = H_em;
        end
        [P_opt_col, ~] = pivot_cal(H_em_frames);
        opt_err = norm(P_opt_col' - P_opt_ref);

        results.(id).em_piv_err   = em_err;
        results.(id).opt_piv_err  = opt_err;
        results.(id).em_piv_pass  = em_err  <= TOL_PIV;
        results.(id).opt_piv_pass = opt_err <= TOL_PIV;

        fprintf('  [%s/%s] dataset-%s  em_err=%.4f mm  opt_err=%.4f mm\n', ...
            pass_str(results.(id).em_piv_pass), ...
            pass_str(results.(id).opt_piv_pass), ...
            id, em_err, opt_err);
    catch ME
        fprintf('  [FAIL] dataset-%s  %s\n', id, ME.message);
    end
end

%% =========================================================
%  SECTION 4: Full PA1 Solve
%  Calls pa1_solve end-to-end, compares all outputs against
%  the reference output1 file, and writes the output file.
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 4: Full PA1 Solve\n');
fprintf('========================================\n');

if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

for i = 1:n_debug
    id       = DEBUG_IDS{i};
    pfx      = [DATA_DIR 'pa1-debug-' id '-'];
    out_path = [OUT_DIR  'pa1-debug-' id '-output1.txt'];

    if ~results.(id).ingest_pass
        fprintf('  [SKIP] dataset-%s  (ingestion failed)\n', id);
        continue;
    end

    try
        [P_em_got, P_opt_got, C_got] = pa1_solve(pfx, out_path);
        [~, P_em_ref, P_opt_ref, C_ref] = output_1_data([pfx 'output1.txt']);

        em_err  = norm(P_em_got  - P_em_ref);
        opt_err = norm(P_opt_got - P_opt_ref);

        N_frames = numel(C_got);
        max_C_err = 0;
        for k = 1:N_frames
            err = abs(C_got{k} - C_ref{k}.C);
            max_C_err = max(max_C_err, max(err(:)));
        end

        pass = (em_err <= TOL_PIV) && (opt_err <= TOL_PIV);
        results.(id).solve_pass  = pass;
        results.(id).solve_em_err  = em_err;
        results.(id).solve_opt_err = opt_err;
        results.(id).solve_C_err   = max_C_err;

        fprintf('  [%s] dataset-%s  em_err=%.4f  opt_err=%.4f  C_max=%.4f mm\n', ...
            pass_str(pass), id, em_err, opt_err, max_C_err);
    catch ME
        fprintf('  [FAIL] dataset-%s  %s\n', id, ME.message);
    end
end

%% =========================================================
%  SUMMARY TABLE
% =========================================================
fprintf('\n========================================\n');
fprintf(' SUMMARY  (tol: C=%.2fmm  Piv=%.2fmm)\n', TOL_C, TOL_PIV);
fprintf('========================================\n');

% --- Pass/fail row ---
fprintf('  %-10s %-8s %-10s %-10s %-8s\n', ...
    'Dataset', 'Ingest', 'EM_Piv', 'Opt_Piv', 'Solve');
fprintf('  %s\n', repmat('-', 1, 52));
for i = 1:n_debug
    id = DEBUG_IDS{i};
    r  = results.(id);
    fprintf('  %-10s %-8s %-10s %-10s %-8s\n', ...
        ['debug-' id], ...
        pass_str(r.ingest_pass), ...
        pass_str(r.em_piv_pass), ...
        pass_str(r.opt_piv_pass), ...
        pass_str(r.solve_pass));
end

% --- Actual error magnitudes ---
fprintf('\n  Error magnitudes (mm):\n');
fprintf('  %-10s %-12s %-12s %-12s\n', 'Dataset', 'C_max', 'EM_Piv', 'Opt_Piv');
fprintf('  %s\n', repmat('-', 1, 50));
for i = 1:n_debug
    id = DEBUG_IDS{i};
    r  = results.(id);
    fprintf('  %-10s %-12s %-12s %-12s\n', ...
        ['debug-' id], ...
        fmt_err(r.C_max_err), ...
        fmt_err(r.em_piv_err), ...
        fmt_err(r.opt_piv_err));
end
fprintf('\n');

%% =========================================================
%  SECTION 5: Unknown Dataset Output Generation
%  Runs pa1_solve on unknown datasets h-k and writes
%  output files to output/. No ground truth available.
% =========================================================
fprintf('\n========================================\n');
fprintf(' SECTION 5: Unknown Dataset Output\n');
fprintf('========================================\n');

UNKNOWN_IDS = {'h','i','j','k'};

for i = 1:numel(UNKNOWN_IDS)
    id       = UNKNOWN_IDS{i};
    pfx      = [DATA_DIR 'pa1-unknown-' id '-'];
    out_path = [OUT_DIR  'pa1-unknown-' id '-output1.txt'];

    try
        [P_em_u, P_opt_u, ~] = pa1_solve(pfx, out_path);
        fprintf('  [DONE] unknown-%s  em=[%.2f %.2f %.2f]  opt=[%.2f %.2f %.2f]\n', ...
            id, P_em_u(1), P_em_u(2), P_em_u(3), ...
            P_opt_u(1), P_opt_u(2), P_opt_u(3));
    catch ME
        fprintf('  [FAIL] unknown-%s  %s\n', id, ME.message);
    end
end

fprintf('\nOutput files written to: %s\n', OUT_DIR);

%% =========================================================
%  LOCAL HELPERS
% =========================================================
function s = pass_str(val)
    if val
        s = 'PASS';
    else
        s = '----';
    end
end

function s = fmt_err(val)
    if isnan(val)
        s = 'N/A';
    else
        s = sprintf('%.4f', val);
    end
end
