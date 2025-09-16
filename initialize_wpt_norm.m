 function [threshold, norm_con, difference_sum_con, difference_sum_art,len_coeff] = initialize_wpt_norm(art,con,num_coeff, samples, maximum, d, length, depth, bin_2, wname)
% make an wpt with my wpt function and take the norms of the
% values to compare it with the other wpts of the signal compare
% the artifacts with the contracts of the
% signals once

%initialize the used arrays
%delta is the number of signal vectors which are used to build the
%comparison norm
delta = 15;
wav_art = zeros(samples, 8000);
wav_con = zeros(samples, 8000);
values_art = zeros(samples, num_coeff);
values_con = zeros(samples, num_coeff);
average_val= zeros(1,num_coeff/2+1);
norm_con= zeros(1,num_coeff/2+1);
difference_con= zeros(samples-delta, num_coeff/2+1);
difference_art= zeros(samples-delta, num_coeff/2+1);
difference_sum_con= zeros(1,samples-delta);
difference_sum_art= zeros(1,samples-delta);


%make the wpt for all vectors
for j =1:samples
[wav_art(j,:),len_coeff] = wpt_custom(art(j,:),length, depth, bin_2, wname);
[wav_con(j,:),len_coeff] = wpt_custom(con(j,:),length, depth, bin_2, wname);

    %build the wpt coefficients at the given depth
    for i = 1:num_coeff
        values_art(j,i) = custom_norm(wav_art(j,(i-1)*len_coeff+1:i*len_coeff), len_coeff);
        values_con(j,i) = custom_norm(wav_con(j,(i-1)*len_coeff+1:i*len_coeff), len_coeff);
    end
%     %sum up the vectors of the high frequencies and build the average of
%     %it
%     values_con(j, num_coeff/2+1) = (sum(values_con(j, num_coeff/2+1: num_coeff))/(num_coeff- (num_coeff/2)+1));
%     values_con(j, num_coeff/2+1) = floor(values_con(j, num_coeff/2+1));
end


%normalize the values to maximum
for i = 1:samples
values_art(i,:) = (maximum* values_art(i,:)/max(values_art(i,:)));
values_con(i,:) = (maximum* values_con(i,:)/max(values_con(i,:)));
end



% build the average value for the norm vector
for i = 1:(num_coeff)
   average_val(i) = values_con(1,i)+values_con(2,i)+values_con(3,i)+values_con(4,i)+values_con(5,i)+values_con(6,i)+values_con(7,i)+values_con(8,i)+values_con(9,i)+values_con(10,i)+values_con(11,i)+values_con(12,i)+values_con(13,i)+values_con(14,i)+values_con(15,i);
   norm_con(i)= (average_val(i)/delta);
end


% build the percentual difference between the wpt norm vector and the
% calculated wpt vectors
% this is for verifieng the correctness of the norm_vector and calculation
% of the threshold
% diff * 100 um prozente zu bekommen
for j = delta+1:samples
    for i = 1:(num_coeff)
    difference_art(j-delta,i) = ((-norm_con(i)+values_art(j,i)+d))/(norm_con(i)+d);
    difference_con(j-delta,i)= ((-norm_con(i)+values_con(j,i)+d))/(norm_con(i)+d);
    end
%     for i = 1:num_coeff
%         difference_con(j-delta,i) = round_c(difference_con(j-delta,i));
%         difference_art(j-delta,i) = round_c(difference_art(j-delta,i));
%     end
end

% sum up the single differences for the single coefficients to get an
% overall difference.
for k =  1:(samples-delta)
    difference_sum_art(k) = sum(abs(difference_art(k,:)));
    difference_sum_con(k) = sum(abs(difference_con(k,:)));
end  




%calculat the threshold - it has to be higher then the max
%contraction difference value and lower than the min artefact difference
%value
max_con = max(difference_sum_con(1,(1:samples-delta)));
min_art = min(difference_sum_art(1,(1:samples-delta)));
threshold =((max_con+min_art)/2);


end




