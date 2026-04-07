function [N,frame_data] = cal_readings_data(path)
    %----------------- DETAILS ------------------------%
    % This function takes in files as strings "" - "calbody" (or right now cells) and spits
    % out data from the "calbody" reading

    %----------------- Inputs -------------------------%
    % path = path from current folder to .txt file
    %----------------- Outputs ------------------------%
    % N = [N_D N_A N_C Nframes] 
    %   N_D = number of optical markers on EM base
    %   N_A = number of optical markers on calibration object
    %   N_C = number of EM markers on calibration object
    %   Nframes = number of data frames
    % frame_data = (N_frame x 1) cell that contains D,A,C for each frame 
    % frame_data{1} = [D_1;A_1;C_1] for first frame etc
    %   D = [Dx_1 Dy_1 Dz_1;
    %        Dx_2 Dy_2 Dz_2;
    %        ...
    %       [Dx_N_D Dy_N_D Dz_N_D]
    %
    %   A = [Ax_1 Ay_1 Az_1;
    %        Ax_2 Ay_2 Az_2;
    %       ...
    %       [Ax_N_A Ay_N_A Az_N_A]
    %
    %   C = [Cx_1 Cy_1 Cz_1;
    %        Cx_2 Cy_2 Cz_2;
    %       ...
    %       [Cx_N_C Cy_N_C Cz_N_C]
    
    %convert file to cell
    cell_array = readcell(path);
    %extract N
    N_D = cell_array{1,1}; N_A = cell_array{1,2}; N_C = cell_array{1,3};
    Nframes = cell_array{1,4};
    N = [N_D N_A N_C Nframes];
    %convert file to matrix - this will (should) ignore first row b/c
    %string
    M = readmatrix(path);
    frame_data = cell(Nframes,1);
    frame_size = N_D + N_C + N_A;
    %for each frame, pull out all frame data
    % loop through frames
    for k = 1:Nframes
        % block indices for this frame
        idx = (k-1)*frame_size + (1:frame_size);
        
        % extract full frame
        M_frame = M(idx,:);
        
        % split into D, A, C
        D = M_frame(1:N_D, :);
        A = M_frame(N_D+1:N_D+N_A, :);
        C = M_frame(N_D+N_A+1:end, :);
        % store (clean structured storage)
        frame_data{k}.D = D;
        frame_data{k}.A = A;
        frame_data{k}.C = C;
    end
end