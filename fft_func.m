function [ fax_Hz, X_mags ] = fft_func( L, fs, emg )

Y = fft(emg);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
X_mags = P1;
fax_Hz = fs*(0:(L/2))/L;

% X_mags = abs(fft(emg));
% 
% bin_vals = [0 : L-1];
% fax_Hz = bin_vals*fs/L;
% N_2 = ceil(L/2);

% plot(fax_Hz(1:N_2), X_mags(1:N_2))
% xlabel('Frequency (Hz)')
% ylabel('Magnitude');
% title('Single-sided Magnitude spectrum (Hertz)');
% axis tight



% hmm some other thougths
% %fft_func Summary of this function goes here
% %   Detailed explanation goes here
% T = 1/fs;
% t = 0:(L-1)*T;
% Y = fft(emg,L);
% Pyy = Y.*conj(Y)/L;
% f = fs*(0:(L/2)-1)/L;
% Pyy = Pyy(1:L/2);
% 
% ff=fft(emg);
% fft1=abs(ff/length(ff));
% %fft1 = fft1(1:length(ff)/2+1);
% %fft1(2:end-1) = 2*fft1(2:end-1);
% freq = fs*(0:(length(ff)-1))/length(ff);
% % figure;
% % plot(freq,fft1);
% % % plot(f,Pyy(1:L/2), farbe)
% % title('Frequency of emg(t)')
% % xlabel('f(Hz)')
% % ylabel('|Power(f)|')

end

