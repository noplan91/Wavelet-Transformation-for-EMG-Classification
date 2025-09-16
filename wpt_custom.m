function [out, len_coeff] = wpt_custom( s_in, len_sig, depth, maximum, wname)

%initializing of the values
%length of the test array
speicher = zeros(1,8000);
%coefficients for wpt
% db4 coefficients
%already multiplicated for the c-implementation for better comparison
%Lo_D = [-1 3 3 -19 -3 65 73 24];
%Hi_D = [-24 73 -65 -3 19 3 3 -1];
[Lo_D, Hi_D] = wfilters(wname,'d');

%Lo_D = 100*Lo_D;
%Hi_D = 100*Hi_D;
%hallo = length(Lo_D);
%for l = 1:1:hallo
%    Lo_D(l) = round_down(Lo_D(l));
%    Hi_D(l) = round_down(Hi_D(l));
%end

%set up variables for the first wavelet transformation
db_len = length(Lo_D);
len_signal = len_sig;
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

% normalize output vector
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
        speicher(2,(j-1)*520+i/2) = cal(1,i);
        speicher(2,(j-1)*520+len_out+i/2) = cdl(1,i);
    end
end
% 
len_coeff = floor(len_input_vec/2);
% normalize output vector
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

% normalize output vector
len_coeff = floor(len_input_vec/2);
speicher(3,:) =floor( maximum*speicher(3,:)/max(speicher(3,:)));
out = speicher(3,:);

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
    
    len_coeff = floor(len_input_vec/2);
    % normalize output vector
    speicher(4,:) =floor( maximum*speicher(4,:)/max(speicher(4,:)));
    %gives out the lowest row of the wpt
    out = speicher(4,:);
    if depth >= 5
    %fourth order wavelets
    %const var ord 4
    len_signal = len_out;
    len_input_vec = len_signal+db_len-1;
    len_out = floor(len_input_vec/2);
    n=1;
    for j = 1:1:16
        cal = convolution(Lo_D, speicher(4,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
        cdl = convolution(Hi_D, speicher(4,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
        for i = 2:2:len_input_vec
            speicher(5,(j-1)*len_input_vec+i/2) = cal(1,i);
            speicher(5,(j-1)*len_input_vec+len_out+i/2) = cdl(1,i);
        end
    end
    
    len_coeff = floor(len_input_vec/2);
    % normalize output vector
    speicher(5,:) =floor( maximum*speicher(5,:)/max(speicher(5,:)));
    %gives out the lowest row of the wpt
    out = speicher(5,:);
        if depth >= 6
        %fourth order wavelets
        %const var ord 4
        len_signal = len_out;
        len_input_vec = len_signal+db_len-1;
        len_out = floor(len_input_vec/2);
        n=1;
        for j = 1:1:32
            cal = convolution(Lo_D, speicher(5,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
            cdl = convolution(Hi_D, speicher(5,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
            for i = 2:2:len_input_vec
                speicher(6,(j-1)*len_input_vec+i/2) = cal(1,i);
                speicher(6,(j-1)*len_input_vec+len_out+i/2) = cdl(1,i);
            end
        end

        len_coeff = floor(len_input_vec/2);
        % normalize output vector
        speicher(6,:) =floor( maximum*speicher(6,:)/max(speicher(6,:)));
        %gives out the lowest row of the wpt
        out = speicher(6,:);
            if depth >= 7
            %fourth order wavelets
            %const var ord 4
            len_signal = len_out;
            len_input_vec = len_signal+db_len-1;
            len_out = floor(len_input_vec/2);
            n=1;
            for j = 1:1:64
                cal = convolution(Lo_D, speicher(6,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
                cdl = convolution(Hi_D, speicher(6,(j-1)*len_signal+1:j*len_signal), db_len, len_signal);
                for i = 2:2:len_input_vec
                    speicher(7,(j-1)*len_input_vec+i/2) = cal(1,i);
                    speicher(7,(j-1)*len_input_vec+len_out+i/2) = cdl(1,i);
                end
            end

            len_coeff = floor(len_input_vec/2);
            % normalize output vector
            speicher(7,:) =floor( maximum*speicher(7,:)/max(speicher(7,:)));
            %gives out the lowest row of the wpt
            out = speicher(7,:);
            end
        end
    end
end
end
end
