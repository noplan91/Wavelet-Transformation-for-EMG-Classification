%close and clear the workspace
clc;
clear all;
close all;

% time counter to check the duration of the function

%% Variables
% 
%initialize the used variables
maximum_tries = [128];           % maximum that is used for normalising
len_sample = 1024;      % tested sample length
cust = 0;               % custom or whole wpt, 0 is whole and is standard, cust not implemented
all_data = 1;           % defines which data read_in file is used
fs = 2000;              % sampling frequency
step = 5;               % step size of the input sampling(step:5 is 2kHz, 10 is 1kHz) 
art_all = [];           % initialize matrix for artefacts
con_all = [];           % initialize matrix for contractions
depth_tries = [3 4 5 6 7];      % set depth of the wpt
d_tries =[1];                  % factor to optimize difference
index_out = 1;
threshold_delta = [1];            % set a multiplier to change the threshold

%%set wavelet
wname_tries = ["db2","db4","db8","db16","coif1","coif2","coif3","coif4","coif5","sym2","sym4","sym5","sym8","bior1.1","bior1.3","bior1.5","bior2.2","bior2.6","bior3.5","bior4.4","bior5.5", "rbio1.5", "rbio4.4","fk8"];
wname_tries = ["db4"];
len_tried_wavs = length(wname_tries);
len_d = length(d_tries);
len_depth = length(depth_tries);
len_max = length(maximum_tries);
len_thr = length(threshold_delta);
for thr_ind = 1:len_thr
threshold_mul = threshold_delta(thr_ind);
for max_ind = 1:1:len_max
maximum = maximum_tries(max_ind);
for index = 1:len_tried_wavs
wname = wname_tries(index);
wname_out = index;
for j = 1:len_depth
depth = depth_tries(j);
for i = 1:len_d
d = d_tries(i);
    

%start time of one wavelet
time_count = 0;
tic;
%% Investigations
% savel/plotting options (on= 1, off= else)
plot_in_signals = 0;    % plot input signals
plot_fft = 0;           % plot the fft
plot_diff = 0;          % plots the difference between the wpt vectors to the wpt norm vector
save_comparison_signals = 0; % save some signals
save_ind = 150;         % index number of the saved signal
output_vec =1;          % output vector with all used variables and a first evaluation of the wptfunction
wpt_vectors = 0;        % print out test vectors
test_vec = 0;        % output of the wpt for testing

%% Initializing the arrays for the Wpt
% read in data and split the data into small arrays with overlapping samples

%read in data to the matrices
if all_data == 1
   [art_all, con_all] = read_in_data( art_all, con_all, step, maximum);
end

%get max length of the samples
samples_art = floor(length(art_all(:,1))/len_sample)-2;
samples_con = floor(length(con_all(:,1))/len_sample)-2;
%split data into small arrays
for i = 1:2:samples_art
    j=(i-1)*len_sample+1;
    t=j + len_sample/2;
    k=i*len_sample;
    p=k+len_sample/2;
    artefact(i,:) = (art_all(j:k,1));
    artefact(i+1,:) = (art_all(t:p,1));
end
for i = 1:2:samples_con
    j=(i-1)*len_sample+1;
    t= j+len_sample/2;
    k=i*len_sample;
    p=k+len_sample/2;
    contraction(i,:) = (con_all(j:k,1));
    contraction(i+1,:)=(con_all(j:k,1));
end
% set variables depending on the depth of the wpt
% len_coeff is the lenght of the coefficients vector
% num_coeff is the number of used coeffiecients
num_coeff =4;
if depth == 3
%     len_coeff = 133;
    num_coeff = 8;
elseif depth == 4
%    len_coeff = 70;
    num_coeff = 16;
elseif depth == 5
    num_coeff = 32;
elseif depth == 6
    num_coeff = 64;
elseif depth == 7
    num_coeff = 128;
end


%% WPT section
% Initialize WPT 
% initializing the norm vector for the comparision of the wpt coeffizients
[threshold, norm_con, diff_con, diff_art, len_coeff] = initialize_wpt_norm( artefact,contraction,num_coeff, samples_con, maximum, d, len_sample, depth, maximum, wname);
threshold = (threshold*threshold_mul);
% evaluation function which will be programmed for the micro controller
% checks the given calculated norm_con for 
for i = 16:samples_art
[value_is_con(1,i), diff_con_ver(1,i), max_sig(1,i)] = eval_signal( threshold, artefact(i,:) , norm_con,maximum, len_coeff,num_coeff, d, len_sample, depth, maximum,wname);
end
for i = 16:samples_con
[value_is_con(2,i),diff_con_ver(2,i), max_sig(2,i)] = eval_signal( threshold, contraction(i,:) , norm_con,maximum, len_coeff,num_coeff, d, len_sample, depth, maximum,wname);
end

%% Testing maximum values of the wpt convolution function
if (test_vec == 1)
    % this artefact and contraction are used in the c-code
    art = [2	1	0	-2	0	3	4	1	0	6	13	15	5	1	1	4	6	5	2	1	-4	-10	-14	-14	-13	-10	-4	-1	-2	-2	3	6	6	0	-5	-7	-9	-9	-7	-6	-6	-4	-2	0	2	4	5	5	4	4	3	0	1	1	0	1	-1	-3	-4	-4	-3	1	1	0	0	2	3	2	0	1	3	5	6	5	7	9	9	8	6	2	0	0	0	-1	0	2	4	5	5	5	4	4	5	4	2	-1	-3	-2	-3	-4	-3	-2	-2	-3	-3	-3	-2	-2	-1	-1	-2	-2	-2	-4	-5	-5	-5	-4	-2	-2	1	1	1	2	2	3	5	5	2	-2	-1	-2	-2	-2	-2	-2	-2	-4	-5	-5	-3	0	0	1	1	1	2	3	6	8	8	7	6	3	-4	-7	-9	-8	-6	-4	-2	0	1	1	-1	-2	-2	0	2	1	-2	-2	-3	-4	-4	-3	-2	-2	-4	-7	-8	-9	-9	-7	-5	-4	-2	-1	-2	-2	-2	-2	-1	1	2	4	5	6	5	4	3	3	3	2	2	3	1	-1	-2	-3	-2	0	1	1	0	0	0	1	2	2	2	1	1	-1	0	2	2	3	2	1	1	1	1	1	0	1	1	0	2	3	4	4	4	3	2	-2	-3	-4	-5	-4	-4	-4	-5	-5	-4	-4	-3	-2	-2	-3	-2	-2	-2	-2	1	2	2	2	2	1	1	1	1	1	-2	-2	-1	0	-2	-3	-5	-6	-6	-6	-6	-6	-6	-6	-7	-7	-8	-8	-8	-8	-7	-6	-5	-5	-4	-3	-3	-3	-3	-8	-10	-11	-11	-11	-11	-10	-10	-8	-7	-5	-5	-4	-4	-4	-4	-4	-4	-4	-4	-3	-2	1	1	2	3	2	1	-3	-5	-6	-7	-8	-8	-8	-8	-8	-8	-3	0	1	1	2	2	2	1	-2	-1	1	1	1	1	1	2	1	0	-2	-3	-4	-5	-6	-5	-4	-3	-2	-1	1	2	3	3	4	4	4	4	4	4	4	5	5	6	7	7	8	9	9	9	8	8	8	7	7	6	5	5	6	6	6	7	7	7	6	6	5	4	4	4	5	5	5	5	5	5	5	5	5	4	3	3	3	3	3	3	2	2	2	2	2	2	2	1	2	1	1	1	1	1	1	1	1	0	0	-2	-2	-2	-3	-3	-3	-3	-4	-4	-4	-4	-4	-4	-4	-4	-4	-4	-3	-3	-3	-3	-3	-3	-3	-3	-2	-2	-2	-2	-2	-2	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-1	-1	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-1	0	0	0	1	1	1	1	1	2	2	1	1	1	1	1	1	0	0	0	0	0	1	1	0	0	1	1	1	1	1	1	1	1	0	0	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	0	0	0	0	0	0	1	1	0	0	0	0	0	0	1	1	1	1	1	1	1	1	1	0	1	0	0	0	0	0	0	0	1	1	1	0	0	0	0	0	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-2	-2	-2	-2	-2	-2	-2	-1	0	0	0	0	0	0	1	1	0	0	0	1	0	0	0	0	0	0	0	0	0	0	1	1	1	1	0	0	0	-2	-2	-2	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-2	-2	-2	-1	-1	-1	-1	-1	-1	-1	-1	-1	-2	-2	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	-1	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1	1	1	1	0	0	0	0	0	0	0	0	0	0	0	-1	-1	-1	-1	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-1	-1	-1	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-2	-1	-2	-1	-1	-1	-2	-2	-2	-2	-2	-1	-1	-1	-1	-2	-2	-1	-1	-1	-1	0	0	0	0	0	0	0	0	0	0	0	0	0	1	1	1	1	0	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	-1	-1	0	0	0	0	0	0	0	0	0	1	0	0	-1	-1	-2	-2	-2	-2	-2	-2	-1	-2	-2	-2	-3	-3	-3	-4	-3	-4	-4	-4	-4	-4	-4	-4	-5	-5	-5	-5	-5	-6	-6	-5	-5	-6	-6	-6	-6	-7	-7	-8	-8	-8	-9	-8	-8	-8	-8	-8	-7	-6	-5	-5	-5	-5	-4	-4	-4	-4	-4	-4	-4	-4	-3	-3	-3	-4	-4	-4	-4	-4	-4	-4	-4	-5	-5	-4	-4	-3	-3	-2	-2	-2	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-3	-2	-2	-2	-2	-2	-2	-2	0	1	2	2	2	2	0	0	2	3	5	7	9	10	11	12	12	12	12	12	11	10	10	9	9	9	7	4	4	4	5	6	6	7	8	7	7	6	6	6	7	8	15	19	20	20	20	21	19	15	11	7	5	2	1	1	-2	2	4	6	5	4	2];
    con = [-3	-3	-2	0	1	2	1	-2	-4	-4	-4	-2	0	1	1	2	2	2	-2	-2	-1	1	-1	-2	-2	-3	-4	-4	-2	-1	-2	-2	-1	1	2	2	2	2	1	-3	-6	-6	-4	-2	0	2	5	8	8	6	2	-1	-3	-3	-3	1	3	5	5	4	3	1	-1	-6	-8	-8	-6	-6	-7	-4	4	6	6	6	5	5	6	9	7	3	-3	-2	-5	-8	-8	-9	-9	-7	-4	-3	-2	-3	-3	-3	2	4	2	2	5	8	8	5	1	-1	-3	-3	-3	-3	-6	-7	-6	-4	-4	-3	4	5	5	2	-2	-4	-5	-4	-2	2	2	4	6	5	5	1	-4	-5	-4	-6	-7	-7	-7	-6	-2	1	4	2	-2	-3	0	4	4	-4	-7	-6	-3	-2	-5	-6	-3	-2	2	3	5	4	3	4	1	-2	-2	-2	-2	-4	-4	-6	-4	2	4	3	4	6	6	5	1	-5	-4	-3	-1	2	2	1	1	2	2	4	3	1	-1	-2	-2	-2	-2	1	2	2	2	3	1	1	-2	-4	-3	-3	-2	3	5	5	6	7	5	4	1	-2	-4	-7	-8	-6	-6	-4	-4	-3	2	3	4	3	2	1	1	4	5	4	4	2	0	-3	-3	-5	-5	-4	-4	-3	-3	-4	-4	-3	-2	-2	-3	-4	-1	1	5	4	3	2	2	1	1	0	2	2	2	-2	-4	-3	-4	-3	-3	-3	-2	-1	-1	-3	-2	-2	-3	-1	1	1	1	-2	-2	-3	-2	3	4	4	1	-3	-6	-7	-7	-7	-7	-6	-1	3	5	5	4	4	3	1	-2	-2	2	2	2	-3	-4	-3	-5	-3	-2	2	1	1	2	1	1	1	0	-2	1	-1	-3	-3	-3	-4	-2	1	2	3	4	5	5	4	3	2	-1	-2	-2	-2	2	2	0	1	3	1	1	1	4	6	7	7	5	2	1	-2	-2	-4	-6	-8	-6	-6	-6	-3	-2	2	6	7	8	6	5	3	-3	-4	-5	-4	-3	-2	-1	-2	0	-2	-3	-3	-1	0	-2	-5	-8	-7	-3	-2	0	0	3	4	4	2	1	-3	-7	-9	-9	-10	-8	-6	2	4	5	4	2	-1	-2	-3	-5	-4	2	2	1	-1	-4	-7	-6	-6	-7	-6	-3	1	4	5	3	0	-1	0	1	-1	-2	-1	-2	-1	2	4	5	5	4	3	0	-2	-2	-2	-3	-3	-1	2	3	5	5	3	3	2	-2	-4	-4	-2	1	3	3	3	3	1	-2	-4	-4	-3	1	2	6	6	5	2	-3	-3	-3	-1	3	3	4	4	4	6	1	-3	-7	-6	-6	-4	-2	0	1	0	1	3	5	5	3	1	1	0	-2	-2	-2	-2	-4	-3	-2	2	1	1	-2	-2	-3	-3	-3	-1	-2	-2	1	5	8	6	4	1	1	-1	-2	-2	-4	-6	-6	-4	-3	-3	-1	1	1	2	2	1	1	2	1	-2	-4	-5	-2	1	1	-3	-5	-4	-3	-1	2	3	2	-1	3	4	7	6	5	5	1	-5	-10	-11	-7	-4	-3	-4	-2	-3	-4	-3	2	2	2	1	-2	-6	-3	2	6	11	11	9	6	2	-2	-10	-15	-17	-14	-5	1	1	2	2	2	-4	-6	-5	-1	0	1	3	4	4	1	3	5	1	-1	0	3	1	-2	-5	-4	-4	-4	-6	-8	-4	1	3	5	4	6	7	5	3	-2	-2	-2	-4	-4	2	6	6	6	3	2	3	5	-4	-4	-3	-2	-3	-4	-4	3	6	6	2	-2	-6	-3	-2	-1	-5	-6	-5	-2	1	3	6	6	4	2	1	2	0	-2	-5	-9	-12	-11	-11	-13	-10	-4	3	6	9	8	9	10	7	4	2	-2	-2	0	0	-2	-7	-11	-7	-5	-4	-3	1	5	6	6	3	2	5	7	6	7	7	6	4	-6	-7	-5	-2	-2	-6	-10	-12	-13	-13	-9	-3	1	4	5	4	4	3	1	1	3	1	-2	0	-1	-2	-4	-5	-6	-6	-5	-7	-9	-7	-4	1	6	9	12	12	7	3	-3	-7	-9	-5	-4	-6	-4	5	6	3	-4	-7	-9	-7	-3	1	3	3	6	9	6	0	-2	-4	-7	-9	-7	-4	-2	4	8	13	13	10	5	4	1	-3	-4	-2	-6	-8	-6	2	-2	-3	-7	-11	-10	-6	5	7	6	8	11	12	9	4	0	1	-5	-8	-10	-6	-2	0	1	5	9	9	7	5	1	-4	-8	-9	-7	-2	5	9	7	-2	-2	-1	-1	-3	-2	1	2	1	-2	-2	-2	-2	-2	-4	-7	-10	-9	-5	2	4	3	3	-2	-3	-3	1	2	5	9	9	4	1	2	1	-5	-7	-8	-8	-6	-3	-2	-4	-2	1	5	10	9	4	-2	-6	-6	-5	-3	-3	-4	-5	-5	-4	-2	-2	1	4	7	6	3	-3	-7	-14	-15	-10	-2	4	8	9	10	9	3	-2	-8	-10	-7	-4	1	4	9	11	7	3	0	-1	1	1	-2	-2	-4	-6	-2	3	5	4	1	-3	3	3	-3	-5	-5	-2	-2	1	6	7	5	1	-3	-7	-8	-8	-7	-6	-6	-6	-4	-4	-3	2	11	18	22	20	14	10	4	-2	-4	-7	-7	-8	-7	-8	-9	-8	-4	5	8	4	-5	-9	-10	-11	-9	-3	3	8	12	16	18	12	3	-6	-14	-19	-23	-22	-15	-7];

    % wpt of the two given vectors for testing
    [speicher, sp] = wpt_max_values( art, len_sample, depth, maximum);
    [speicher1, sp_1] = wpt_max_values( con, len_sample, depth, maximum);

end
%% Saving Section

% save the used vectors to a txt file for easier comparison to the
% c-pogramm
if (save_comparison_signals ==1)
    dlmwrite('test_data_vectors_art.txt',artefact(save_ind,:),'delimiter',',');
    dlmwrite('test_data_vectors_con.txt',contraction(save_ind,:),'delimiter',',');
    if(wpt_vectors ==1)
        dlmwrite('test_data_first_con.txt',sp,'delimiter',',');
        dlmwrite('test_data_first_art.txt',sp_1,'delimiter',',');
    end
end

%plot the input signals
if (plot_in_signals == 1)
    plot_signals(art_all, con_all,'Input signals', 'input_signals.png', 'x', 'Amplitude', 'artefact', 'contraction',2);
end

% plot fft of the input signals
if(plot_fft == 1)
     plot_fft_func(art_all, con_all,maximum, fs);
end

% plot and save signals
if(plot_diff == 1)
    plot_signals(diff_art, diff_con, 'Difference form the WPT vectors to the WPT norm vector','difference_wpt_to_norm.png', 'x [samples]', 'Difference to the WPT norm vector', 'artefact', 'contraction',2);
    plot_signals(diff_con_ver(1,:), diff_con_ver(2,:), 'Difference form the WPT vectors to the WPT norm vector for verifying','difference_wpt_to_norm_verify.png', 'x [samples]', 'Difference to the WPT norm vector', 'artefact', 'contraction',2);
    plot_signals(max_sig(1,:), max_sig(2,:), 'Maximum value of the wpt coeff','max_value_wpt_coeff.png', 'x [samples]', 'Difference to the WPT norm vector', 'artefact', 'contraction',2);
    plot_signals(norm_con, norm_con,'Norm WPT comparison signal', 'wpt_norm_signal.png', '0< x[Hz] < 1000 ', 'Amplitude', 'contraction norm', '___',1);
end

% get a sumup of the used values and a first calculation of the errors
time_count = time_count +toc;
if (output_vec ==1)
%     output_sum_up = zeros(index,12);
    output_sum_up(index_out,1) = sum(value_is_con(1,:));                    % sum of the found false contractions 
    output_sum_up(index_out,2) = sum(value_is_con(2,:));                    % sum of the found contractions
    output_sum_up(index_out,3) = depth;                                     % used depth of the wpt
    output_sum_up(index_out,4) = d;                                         % used correction factor for the percentual difference
    output_sum_up(index_out,5) = threshold;                                 % used threshold
    output_sum_up(index_out,6) = threshold_mul;                                      % used wpt_function -> is 0 -> my custom function
    output_sum_up(index_out,7) = all_data;                                  % which data is used
    output_sum_up(index_out,8) = maximum;                                   % correction factor of the values, not used.
    output_sum_up(index_out,9)= output_sum_up(index_out,1)/samples_art;             % percentual failure of the artefacts (false negativ)
    output_sum_up(index_out,10)= (samples_con-output_sum_up(index_out,2))/samples_con;        % percentual failure of the contractions (false positiv)
    output_sum_up(index_out,11)= output_sum_up(index_out,9) + output_sum_up(index_out,10);  % whole percentual failure
    output_sum_up(index_out,12)= samples_art-output_sum_up(index_out,1);            % number of right artefacts
    output_sum_up(index_out,13) = time_count;                               % time the whole programm needs
    output_sum_up(index_out,14) = samples_addddlskdlfdsjfrt;
    output_sum_up(index_out,15) = samples_con;
    output_sum_up(index_out,16) = wname_out;
    index_out
    index_out = index_out +1;
    
end
end
end
end
end
end

y_signal = output_sum_up(:,11);
length(y_signal)
plot_signal_with_labels(output_sum_up(:,5),depth_tries, 'Tiefe')
%plot_grouped_signals(y_signal,wname_tries,maximum_tries)



