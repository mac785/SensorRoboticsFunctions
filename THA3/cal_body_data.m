function [N,d,a,c] = cal_body_data(path)
    %----------------- DETAILS ------------------------%
    % This function takes in files as strings "" - "calbody" (or right now cells) and spits
    % out data from the "calbody" reading

    %----------------- Inputs -------------------------%
    % path = path from current folder to .txt file
    %----------------- Outputs ------------------------%
    % N = [Nd Na Nc] 
    %   Nd = number of optical markers on EM base
    %   Na = number of optical markers on calibration object
    %   Nc = number of EM markers on calibration object
    % d = [dx_1 dy_1 dz_1;
    %      dx_2 dy_2 dz_2;
    %      ...
    %     [dx_Nd dy_Nd dz_Nd] 
    %
    % a = [ax_1 ay_1 az_1;
    %      ax_2 ay_2 az_2;
    %      ...
    %     [ax_Na ay_Na az_Na] 
    %
    % c = [cx_1 cy_1 cz_1;
    %      cx_2 cy_2 cz_2;
    %      ...
    %     [cx_Nc cy_Nc cz_Nc] 
    
    %convert file to cell
    cell_array = readcell(path);
    %extract N
    Nd = cell_array{1,1}; Na = cell_array{1,2}; Nc = cell_array{1,3};
    N = [Nd Na Nc];
    %convert file to matrix - this will (should) ignore first row b/c
    %string
    M = readmatrix(path);
    %extract d, a, c
    d = M((1:Nd),:);
    a = M((Nd+1:Nd + Na),:);
    c = M((Nd + 1 + Na:Nd + Na+ Nc),:);
end