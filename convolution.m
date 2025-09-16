function [ Y ] = convolution( x,h, len_x, len_h,depth)
%convolution function which convolutes x with h and has the output
%parameter Y

% variable init
X=[x,zeros(1,len_h)]; 
H=[h,zeros(1,len_x)]; 
Y = zeros(1,len_h+len_x-1);

% discrete convolution
for i=1:len_x+len_h-1
    Y(i)=0;
    for j=1:len_x
        if(i-j+1>0)
            Y(i)=Y(i)+floor((X(j))*((H(i-j+1))));
        else
            %do nothing
        end
    end
end
Y = floor(Y);

end

