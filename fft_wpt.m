function [ h, werte, y_a ] = fft_wpt( emg, depth, wname,  farbe, fs)
%fft_wpt Selfmade fft wpt functino
%   The function takes all the norms form the packets of the
%   wavelet packet tree and saves them into a vector and then
%   plots the power to the frequenzy of the emg input signal
h = wpdec(emg, depth, wname);
for i=1:(2^depth);
    y_a(i) = ((fs/2)/2^(depth))*(i);
    werte(i) = norm(wpcoef(h,[1 i+(2^depth-2)-1]));
    
end
% plot(y_a,werte, farbe);
% grid on;
% hold on;
% title('Amplitude Spectrum of EMG(t)')
% xlabel('f (Hz)')
% ylabel('|Power(f)|')
end

