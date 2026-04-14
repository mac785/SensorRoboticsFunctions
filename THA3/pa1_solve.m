function [P_em, P_opt, C_expected] = pa1_solve(pfx, out_path)
    %----------------- DETAILS ------------------------%
    % Full PA1 pipeline. Loads all input files for a dataset, computes:
    %   - Expected EM marker positions (C_expected) for each cal frame
    %   - EM pivot calibration (P_em)
    %   - Optical pivot calibration (P_opt)
    % Optionally writes an output1 file.
    %
    %----------------- Inputs -------------------------%
    % pfx      = path prefix for the dataset, e.g. 'HW3-PA1/pa1-debug-a-'
    %            Files loaded: <pfx>calbody.txt, calreadings.txt,
    %                          empivot.txt, optpivot.txt
    % out_path = (optional) path to write output1 file.
    %            Pass '' or omit to skip writing.
    %
    %----------------- Outputs ------------------------%
    % P_em       = 1x3 EM pivot post position (in EM base frame)
    % P_opt      = 1x3 optical pivot post position (in EM base frame)
    % C_expected = N_frames x 1 cell array
    %   C_expected{k} = N_c x 3 matrix of expected EM marker positions

    if nargin < 2
        out_path = '';
    end

    %----------------------------------------------------
    % 1. Load data
    %----------------------------------------------------
    [N_body, d, a, c]       = cal_body_data   ([pfx 'calbody.txt']);
    [N_read, frame_data]    = cal_readings_data([pfx 'calreadings.txt']);
    [~, ~, G_frames]        = empivot_data    ([pfx 'empivot.txt']);
    [~, ~, ~, opt_frames]   = optpivot_data   ([pfx 'optpivot.txt']);

    N_frames = N_read(4);
    N_c      = N_body(3);

    %----------------------------------------------------
    % 2. Compute C_expected for each calibration frame
    %    C_i^exp = F_D^{-1} * F_A * c_i
    %----------------------------------------------------
    C_expected = cell(N_frames, 1);
    for k = 1:N_frames
        F_D     = cloud2cloud(d, frame_data{k}.D);
        F_A     = cloud2cloud(a, frame_data{k}.A);
        F_D_inv = inv_transform(F_D);

        C_exp = zeros(N_c, 3);
        for j = 1:N_c
            pt = F_D_inv * (F_A * [c(j,:)'; 1]);
            C_exp(j,:) = pt(1:3)';
        end
        C_expected{k} = C_exp;
    end

    %----------------------------------------------------
    % 3. EM pivot calibration
    %    G_frames are already in the EM base frame.
    %----------------------------------------------------
    [P_em_col, ~] = pivot_cal(G_frames);
    P_em = P_em_col';  % return as 1x3 row

    %----------------------------------------------------
    % 4. Optical pivot calibration
    %    Transform H marker positions from optical frame to EM frame
    %    using F_D^{-1} before running pivot_cal.
    %----------------------------------------------------
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
    P_opt = P_opt_col';  % return as 1x3 row

    %----------------------------------------------------
    % 5. Optionally write output file
    %----------------------------------------------------
    if ~isempty(out_path)
        % ensure output directory exists
        out_dir = fileparts(out_path);
        if ~isempty(out_dir) && ~exist(out_dir, 'dir')
            mkdir(out_dir);
        end
        write_output1(out_path, P_em, P_opt, C_expected);
    end
end
