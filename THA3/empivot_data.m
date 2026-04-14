function [N_G, N_frames, G_frames] = empivot_data(path)
    %----------------- DETAILS ------------------------%
    % Reads a PA1 empivot .txt file and returns EM probe marker
    % positions organized by frame.
    %
    %----------------- Inputs -------------------------%
    % path = path to empivot .txt file
    %
    %----------------- Outputs ------------------------%
    % N_G     = number of EM markers on probe
    % N_frames = number of pivot frames
    % G_frames = N_frames x 1 cell array
    %   G_frames{k} = N_G x 3 matrix of EM marker positions for frame k
    %                 [Gx_1 Gy_1 Gz_1;
    %                  Gx_2 Gy_2 Gz_2;
    %                  ...
    %                  Gx_NG Gy_NG Gz_NG]

    cell_array = readcell(path);
    N_G      = cell_array{1,1};
    N_frames = cell_array{1,2};

    M = readmatrix(path, 'NumHeaderLines', 1);

    G_frames = cell(N_frames, 1);
    for k = 1:N_frames
        idx = (k-1)*N_G + (1:N_G);
        G_frames{k} = M(idx, :);
    end
end
