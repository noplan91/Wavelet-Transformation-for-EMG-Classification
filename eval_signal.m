function [ is_con, difference_sum_signal, max_sig] = eval_signal( threshold, signal_to_check , norm_con,maximum, len_coeff, num_coeff, d, length, depth, bin_2,wname)
%   check if the signal_to_check is a contraction and return 1 if so

    %initialize used vectors
    coeff_signal = zeros(1, num_coeff);
    difference_signal = zeros (1, num_coeff/2+1);
    
    %make the wpt for the signal_to_check
    [wav_signal(1,:)] = wpt_custom(signal_to_check(1,:),length, depth, bin_2,wname);
    max_sig = max(abs(wav_signal(1,:)));
    
    %build the wpt coefficients at the given depth and signal
    for i = 1:num_coeff
        coeff_signal(1,i) = custom_norm(wav_signal(1,(i-1)*len_coeff+1:i*len_coeff), len_coeff);
    end
    
%     %sum up the vectors of the high frequencies and build the average of
%     %it
%     coeff_signal(1, num_coeff/2+1) = (sum(coeff_signal(1, num_coeff/2+1: num_coeff))/(num_coeff- (num_coeff/2)+1));
%     coeff_signal(1, num_coeff/2+1) = floor(coeff_signal(1, num_coeff/2+1));

    %normalize the values to maximum
    coeff_signal(1,:) = (maximum* coeff_signal(1,:)/max(coeff_signal(1,:)));
    % build the percentual difference between the wpt norm vector and the
    % calculated wpt vectors
    % diff * 100 um prozente zu bekommen
    for i = 1:(num_coeff)
    difference_signal(1,i) = ((-norm_con(i)+coeff_signal(1,i)+d))/(norm_con(i)+d);
    end
    for i = 1:num_coeff/2+1
        difference_signal(1,i) = round_c(difference_signal(1,i));
    end

    % sum up the single differences for the single coefficients to get an
    % overall difference.
    difference_sum_signal = sum(abs(difference_signal(1,:)));
    %sum up the koefficients and check if the value is below or above the
    %threshold
    is_con = 0;
    if difference_sum_signal <= threshold
        is_con= is_con+1;
    end
    
end




