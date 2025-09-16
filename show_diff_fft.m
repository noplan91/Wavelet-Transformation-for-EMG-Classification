close all;
clear all;
clc;

fs = 2000;              % sampling frequency
step = 5;               % step size of the input sampling(step:5 is 2kHz, 10 is 1kHz) 
art_all = [];           % initialize matrix for artefacts
con_all = [];           % initialize matrix for contractions
maximum = 32;

[art_all, con_all] = read_in_data( art_all, con_all, step, maximum);
samples = 350;
for i = 1:samples
    j=(i-1)*1024+1;
    k=i*1024;
    a(i,:) = art_all(j:k,1);
    b(i,:) = con_all(j:k,1);
end

L2 = 359035;
L1 = 569650;
[ f1, Pyy_art_all ] = fft_func( L1, fs, art_all');
[ f2, Pyy_con_all ] = fft_func( L2, fs, con_all');

werte_art_all = maximum* Pyy_art_all/max(Pyy_art_all);
werte_con_all = maximum* Pyy_con_all/max(Pyy_con_all);
i = 1;
figure;
plot(f1(1:L1/2),werte_art_all(i,1:L1/2),f2(1:L2/2),werte_con_all(i,1:L2/2),'--');
%axis([0 1000 0 1]);
title('FFT of the EMG Signals');
xlabel('Hz');
ylabel('Amplitude');
legend('relaxation','contraction','artefact');


% make an fft of the sample signals to test if the wpt is correct
L = 1024;
for i = 1:samples
[ f1, Pyy_art(i,:) ] = fft_func( L, fs, a(i,:));
[ f1, Pyy_con(i,:)] = fft_func( L, fs, b(i,:));
end
%plot the fft of the signals
% normalize the vectors to maximum
% and plot a art, con and rel in one plot
for i = 1:samples;
werte_art(i,:) = maximum* Pyy_art(i,:)/max(Pyy_art(i,:));
werte_con(i,:) = maximum* Pyy_con(i,:)/max(Pyy_con(i,:));
end
% for i = 1:1
% figure;
% plot(f1(1:L/2),werte_art(i,1:L/2),f1(1:L/2),werte_con(i,1:L/2),'--',f1(1:L/2) ,werte_rel(i,1:L/2));
% axis([0 1000 0 1]);
% title('FFT of the EMG Signals');
% legend('artefact','contraction','relaxation');
% end;



mittelwert_cust = 0;
for i = 1:L/2
    mittelwert_cust(i) = werte_con(1,i)+werte_con(2,i)+werte_con(3,i)+werte_con(4,i)+werte_con(5,i)+werte_con(6,i)+werte_con(7,i)+werte_con(8,i)+werte_con(9,i)+werte_con(10,i);
    norm_cust_con(i)= mittelwert_cust(i)/10;
end

%take the difference between the norm and the other split samples
for j = 11:samples
    for i = 1:L/2
    difference_cust_art(j-10,i) = -norm_cust_con(i)+werte_art(j,i);
    difference_cust_con(j-10,i)= -norm_cust_con(i)+werte_con(j,i);
    end
end;
k=1;

for k =  1:(samples-10)
    difference_cust_all_art(k) = sum(abs(difference_cust_art(k,:)));
    difference_cust_all_con(k) = sum(abs(difference_cust_con(k,:)));
end  
m = 1:1:(samples-10);
figure;
plot(m,difference_cust_all_art,m,difference_cust_all_con);
legend('diff cust art','diff cust con');
%axis([0 samples-10 0 150]);
wname = 'fft';
xlabel('Samples');
ylabel('Amplitude');
l = ['difference with the ',sprintf(wname)];
title(l);

%get the errors
max_con = max(difference_cust_all_con(1,(1:90)));
min_art = min(difference_cust_all_art(1,(1:90)));
threshold = (max_con+min_art)/2;
error = 0;
for i = 1:250
    if difference_cust_all_art(1,90+i) <= threshold
        error= error+1;
    elseif difference_cust_all_con(1,90+i) >= threshold
        error= error+1;
    end
end
l
threshold
error