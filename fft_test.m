 %% Fourier Script
% This is a script that uses the fft to show the parts of the
% signals with an fft to see the difference between them.
%%
%clear all;
src1 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_10kg_1.mat');
src2 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_15kg_1.mat');
src3 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_30kg_1.mat');
src4 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_40kg_1.mat');
src5 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_series.mat');
src6 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_entspannt_1.mat');
src7 = load('C:\Users\Nutzer\Documents\Bachelorarbeit\messungen_michi\kurt_artefact_1.mat');
emg1 = src1.src1.Data(1,:);
emg1 = emg1(20000:5:60000);
emg1 = emg1-mean(emg1);
emg2 = src2.src1.Data(1,:);
emg2 = emg2(20000:5:60000);
emg2 = emg2-mean(emg2);
emg3 = src3.src1.Data(1,:);
emg3 = emg3(20000:5:60000);
emg3 = emg3-mean(emg3);
emg4 = src4.src1.Data(1,:);
emg4 = emg4(20000:5:60000);
emg4 = emg4-mean(emg4);
emg5 = src5.src1.Data(1,:);
emg5 = emg5(20000:5:60000);
emg5 = emg5-mean(emg5);
emg6 = src6.src1.Data(1,:);
emg6 = emg6(20000:5:60000);
emg6 = emg6 - mean(emg6);
emg7 = src7.src1.Data(1,:);
emg7 = emg7(20000:5:60000);
emg7 = emg7 - mean(emg7);
 figure;
 hold on;
 grid on;
 plot(emg1, 'y');
 plot(emg2, 'm');
 plot(emg3, 'c');
 plot(emg4,'r');
 plot(emg5, 'g');
 plot(emg6, 'b');
 plot(emg7, 'k');
 
fs = 2000;
L = 40000;
T = 1/fs;
t = 0:(L-1)*T;
Y = fft(emg1,L);
Pyy = Y.*conj(Y)/L;
f = fs*(0:(L/2)-1)/L;
figure;
grid on;
hold on;
plot(f,Pyy(1:L/2), 'y')
Y2 = fft(emg2,L);
Pyyy = Y2.*conj(Y2)/L;
f2 = fs*(0:(L/2)-1)/L;
plot(f2,Pyyy(1:L/2), 'm');
Y3 = fft(emg3,L);
Py3 = Y3.*conj(Y3)/L;
f3 = fs*(0:(L/2)-1)/L;
plot(f3,Py3(1:L/2), 'c');
Y4 = fft(emg4,L);
Py4 = Y4.*conj(Y4)/L;
f4 = fs*(0:(L/2)-1)/L;
plot(f4,Py4(1:L/2), 'r');
Y5 = fft(emg5,L);
Py5 = Y5.*conj(Y5)/L;
f5 = fs*(0:(L/2)-1)/L;
plot(f5,Py5(1:L/2), 'g');
Y6 = fft(emg6,L);
Py6 = Y6.*conj(Y6)/L;
f6 = fs*(0:(L/2)-1)/L;
plot(f6,Py6(1:L/2), 'b');
Y7 = fft(emg7,L);
Py7 = Y7.*conj(Y7)/L;
f7 = fs*(0:(L/2)-1)/L;
plot(f7,Py7(1:L/2), 'k');
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|Power(f)|')
figure;
hold on;
time_f = 4000;
plot(f(1:time_f),Pyy(1:time_f), 'r');
hold on;
plot(f3(1:time_f),Py3(1:time_f), 'g');
plot(f5(1:time_f),Py5(1:time_f), 'b');
plot(f7(1:time_f),Py7(1:time_f), 'k');
title('Power spectral density')
xlabel('Frequency (Hz)')
