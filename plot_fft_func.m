 function [] = plot_fft_func(art_all, con_all,maximum, fs)
% make an fft of the sample signals to test if the wpt is correct
[ f1, Pyy_art ] = fft_func( length(art_all), fs, art_all');
[ f2, Pyy_con] = fft_func( length(con_all), fs, con_all');
%plot the fft of the signals
% normalize the vectors to maximum
% and plot a art, con and rel in one plot
for i = 1:1
Pyy_art(i,:) = maximum* Pyy_art(i,:)/max(Pyy_art(i,:));
Pyy_con(i,:) = maximum* Pyy_con(i,:)/max(Pyy_con(i,:));  
end

figure;
plot(f2(1:length(con_all)/2-2), Pyy_con(1,1:length(con_all)/2-2),f1(1:length(art_all)/2-1), Pyy_art(1,1:length(art_all)/2-1));
art_1 = 'artefact ';
con_1 = 'contraction ';
art_title =  [art_1];
con_title = [con_1];
% axis([0 1000 0 maximum]);
tit = ['FFT of the EMG Input Signals'];
title(tit);
legend(con_title, art_title);
xlabel('0 < x/Hz < 1000') % x-axis label
ylabel('Amplitude') % y-axis label
hold on;
saveas(gcf,'FFT_input_signals.png')

clc;