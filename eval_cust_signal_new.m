function [ is_con, difference_cust ] = eval_cust_signal_new( threshold, signal_to_check , norm_cust_con,maximum, d1, d3, d, length, depth, bin_2)
%end_check
%   check if the signal_to_check is a contraction and return 1 if so

    %initialize variables
    value_cust= zeros(1,d3/2+2);
    difference_cust = zeros(1,d3/2+2);
    j = 1;
    %wpt of the signal to check
    wav_a = zeros(1,3000);
    [wav_a(j,:)] = wpt_whole_tree(signal_to_check(j,:), length, depth, bin_2);
    re=range (wav_a);
    d2=262;
    %get the wavelet coeffizients form the bottom level
    for i = 1:d3/2
       value_cust(j,i) = custom_norm(wav_a(j,(i-1)*d1+1:i*d1),d1);
    end
    for i = 1:2
        value_cust(j,d3/2+i) = custom_norm(wav_a(j,1500+(d2*(i-1)):1762+(d2*(i-1))),d2);
    end
    
    %normalize the values
    value_cust(j,:) = (maximum* value_cust(j,:)/max(value_cust(j,:)));
    %get the percentual difference between the signal_to_check and a given
    %norm
    for i = 1:(d3/2+2)
    difference_cust(j,i) = ((-norm_cust_con(i)+value_cust(j,i)+d)/(norm_cust_con(i)+d));
    end
    
    
    %sum up the koefficients and check if the value is below or above the
    %threshold
    is_con = 0;
    difference_cust_all = sum(abs(difference_cust(j,:)));
    if difference_cust_all <= threshold
        is_con= is_con+1;

    end
    
    
    
end




