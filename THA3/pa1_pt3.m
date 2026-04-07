%% ----------- ADJUST THIS -----------
clear
% Provide cal_body_data path and cal_readings_data path from current folder
body_path = "calbody/pa1-debug-b-calbody.txt";
reading_path = "calreadings/pa1-debug-b-calreadings.txt";
%-------- SOLVE ------
% Extract data from files
[N_body,d,a,c] = cal_body_data(body_path);
[N_readings,frame_data] = cal_readings_data(reading_path);
Nframes = N_readings(4);
N_c = N_body(3);
%solve for C_i_expected
F_D_array = cell(Nframes,1);
F_A_array = cell(Nframes,1);
C_i_exp_array = cell(Nframes,1);
error_array = cell(Nframes,1);
for i = 1:Nframes
    %First solve for F_d
    D_readings = frame_data{i}.D;
    d_body = d;
    F_D = cloud2cloud(d,D_readings);
    F_D_array{i} = F_D;

    %solve for F_a
    A_readings = frame_data{i}.A;
    a_body = a;
    F_A = cloud2cloud(a_body,A_readings);
    F_A_array{i} = F_A;

   %solve for C_i_expected
   C_i_exp_matrix = zeros(N_c,4);
   for k = 1:N_c
       c_i = c(k,:);
       C_i_exp_matrix(k,:) = (inv(F_D)*F_A*[c_i 1]')';
   end
   C_i_exp_array{i} = C_i_exp_matrix(:,1:3);

   error_array{i} = mean(frame_data{i}.C - C_i_exp_matrix(:,1:3),1);
end