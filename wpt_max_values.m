function [speicher, speicher_1] = wpt_max_values( s_in, length, depth, maximum)

%initializing of the values
%length of the test array
speicher = zeros(1,1120);
wav_out = zeros(1,1120);
speicher_1 = zeros(4,1120);
%coefficients for wpt
% db4 coefficients
%already multiplicated for the c-implementation for better comparison
% and smaller range of amplitude
Lo_D = [-1 3 3 -19 -3 65 73 24];
Hi_D = [-24 73 -65 -3 19 3 3 -1];

%set up variables for the first wavelet transformation
db_len = length(Lo_D);
len_signal = length;
len_input_vec = len_signal+db_len-1;
len_out = floor(len_input_vec/2);
%first order wavelets
n=1;
for j = 1:1:1
    cal = convolution(Lo_D, s_in, db_len, len_signal);
    cdl = convolution(Hi_D, s_in, db_len, len_signal);
    for i = 2:2:len_input_vec
        speicher(1,i/2) = cal(1,i);
        speicher(1,len_out+i/2) = cdl(1,i);
    end
end

speicher_1(1,:) = speicher;
temp = max(speicher(1,:));
speicher(1,:) =(32*speicher(1,:))/temp;
speicher(1,:) = floor(speicher(1,:));
%second order wavelets
%const var ord 2
len_signal = len_out;
len_input_vec = len_signal+db_len-1;
len_out = floor(len_input_vec/2);
n=1;
for j = 1:1:2
    cal = convolution(Lo_D, speicher(1,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
    cdl = convolution(Hi_D, speicher(1,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
    for i = 2:2:len_input_vec
        speicher(2,(j-1)*len_input_vec+i/2) = cal(1,i);
        speicher(2,(j-1)*len_input_vec+len_out+i/2) = cdl(1,i);
    end
end
% 
speicher_1(2,:) = speicher(2,:);
speicher(2,:) =floor( maximum*speicher(2,:)/max(speicher(2,:)));
if depth >= 3
%third order wavelets
%const var ord 3
    len_signal = len_out;
    len_input_vec = len_signal+db_len-1;
    len_out = floor(len_input_vec/2);
n=1;
for j = 1:1:4
    cal = convolution(Lo_D, speicher(2,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
    cdl = convolution(Hi_D, speicher(2,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
    for i = 2:2:len_input_vec
        speicher(3,(j-1)*len_input_vec+i/2) = cal(1,i);
        speicher(3,(j-1)*len_input_vec+len_out+i/2) = cdl(1,i);
    end
end

speicher_1(3,:) = speicher(3,:);
speicher(3,:) =floor( maximum*speicher(3,:)/max(speicher(3,:)));

if depth >= 4
    %fourth order wavelets
    %const var ord 4
    len_signal = len_out;
    len_input_vec = len_signal+db_len-1;
    len_out = floor(len_input_vec/2);
    n=1;
    for j = 1:1:8
        cal = convolution(Lo_D, speicher(3,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
        cdl = convolution(Hi_D, speicher(3,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
        for i = 2:2:len_input_vec
            speicher(4,(j-1)*len_input_vec+i/2) = cal(1,i);
            speicher(4,(j-1)*len_input_vec+len_out+i/2) = cdl(1,i);
        end
    end
    speicher_1(4,:) = speicher(4,:);
    speicher(4,:) =floor( maximum*speicher(4,:)/max(speicher(4,:)));
end
end
end
