function [N_D, N_H, N_frames, frame_data] = optpivot_data(path)
    %----------------- DETAILS ------------------------%
    % Reads a PA1 optpivot .txt file and returns optical marker positions
    % for both the EM-base trackers (D) and probe trackers (H), per frame.
    %
    %----------------- Inputs -------------------------%
    % path = path to optpivot .txt file
    %
    %----------------- Outputs ------------------------%
    % N_D      = number of optical markers on EM base
    % N_H      = number of optical markers on probe
    % N_frames = number of pivot frames
    % frame_data = N_frames x 1 cell array
    %   frame_data{k}.D = N_D x 3 matrix of EM-base optical marker positions
    %   frame_data{k}.H = N_H x 3 matrix of probe optical marker positions
    %
    %   Per frame layout in file (N_D rows then N_H rows):
    %     [Dx_1 Dy_1 Dz_1;  ...;  Dx_ND Dy_ND Dz_ND]   <- EM base trackers
    %     [Hx_1 Hy_1 Hz_1;  ...;  Hx_NH Hy_NH Hz_NH]   <- probe trackers

    cell_array = readcell(path);
    N_D      = cell_array{1,1};
    N_H      = cell_array{1,2};
    N_frames = cell_array{1,3};

    M = readmatrix(path); % ignores header row

    frame_size = N_D + N_H;
    frame_data = cell(N_frames, 1);
    for k = 1:N_frames
        idx = (k-1)*frame_size + (1:frame_size);
        M_frame = M(idx, :);
        frame_data{k}.D = M_frame(1:N_D, :);
        frame_data{k}.H = M_frame(N_D+1:end, :);
    end
end
