function [N, P_em, P_opt, frame_data] = output_1_data(path)
    %----------------- DETAILS ------------------------%
    % This function reads a PA1 Problem 1 output .txt file
    % and extracts:
    %   - N_c and Nframes
    %   - EM pivot point
    %   - Optical pivot point
    %   - Expected C marker positions for each frame
    %
    %----------------- Inputs -------------------------%
    % path = path from current folder to output .txt file
    %
    %----------------- Outputs ------------------------%
    % N = [N_c Nframes]
    %   N_c = number of EM markers on calibration object
    %   Nframes = number of frames
    %
    % P_em = [Px Py Pz]
    %   EM pivot calibration position
    %
    % P_opt = [Px Py Pz]
    %   Optical pivot calibration position
    %
    % frame_data = Nframes x 1 cell array
    %   frame_data{k}.C = N_c x 3 matrix of expected C marker positions
    %
    % Example:
    %   [N, P_em, P_opt, frame_data] = output_1_data("pa1-output.txt");

    %---------------- Read file ----------------%
    cell_array = readcell(path);

    %---------------- Extract header ------------%
    N_c = cell_array{1,1};
    Nframes = cell_array{1,2};

    N = [N_c Nframes];

    %---------------- Extract pivot points ------%
    P_em = [cell_array{2,1}, cell_array{2,2}, cell_array{2,3}];
    P_opt = [cell_array{3,1}, cell_array{3,2}, cell_array{3,3}];

    %---------------- Extract frame data --------%
    M = readmatrix(path);

    % remove first 3 rows:
    % row 1 = header
    % row 2 = P_em
    % row 3 = P_opt
    C_all = M(4:end,:);

    % preallocate
    frame_data = cell(Nframes,1);

    % each frame has N_c rows
    frame_size = N_c;

    % loop through frames
    for k = 1:Nframes

        % block indices for this frame
        idx = (k-1)*frame_size + (1:frame_size);

        % extract current frame C data
        C = C_all(idx,:);

        % store
        frame_data{k}.C = C;
    end
end