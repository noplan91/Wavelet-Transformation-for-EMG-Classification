 function [norm_c] = custom_norm(s_in , len_n)
 % function to build the norm of a vector s_in with given lenght len_n
 
 %variable init
 norm_c = 0;
 tmp = 0;
 
 %norm
 for i = 1:1:len_n
      tmp =tmp+ s_in(i)^2;
 end
 norm_c = sqrt(tmp);
  
 norm_c = floor(norm_c);
 end
 