function write_output1(path, P_em, P_opt, C_expected)
    %----------------- DETAILS ------------------------%
    % Writes a PA1 output1 .txt file in the format expected by output_1_data.m
    %
    %----------------- Inputs -------------------------%
    % path       = output file path (string)
    % P_em       = 1x3 EM pivot post position
    % P_opt      = 1x3 optical pivot post position
    % C_expected = N_frames x 1 cell array
    %   C_expected{k} = N_c x 3 matrix of expected EM marker positions for frame k
    %
    %----------------- Output file format --------------%
    % Line 1:  N_c, N_frames, <filename>
    % Line 2:  P_em  (x, y, z)
    % Line 3:  P_opt (x, y, z)
    % Lines 4+: N_c rows per frame, all frames concatenated

    N_frames = double(numel(C_expected));
    N_c      = double(size(C_expected{1}, 1));

    % extract just the filename for the header
    [~, fname, ext] = fileparts(path);
    header_name = char([fname ext]);

    fid = fopen(path, 'w');
    if fid == -1
        error('write_output1: could not open file: %s', path);
    end

    % header line
    fprintf(fid, '%d, %d, %s\n', N_c, N_frames, header_name);

    % pivot positions
    fprintf(fid, '%8.2f, %8.2f, %8.2f\n', P_em(1),  P_em(2),  P_em(3));
    fprintf(fid, '%8.2f, %8.2f, %8.2f\n', P_opt(1), P_opt(2), P_opt(3));

    % expected C positions — all frames concatenated
    for k = 1:N_frames
        C = C_expected{k};
        for i = 1:N_c
            fprintf(fid, '%8.2f, %8.2f, %8.2f\n', C(i,1), C(i,2), C(i,3));
        end
    end

    fclose(fid);
end
