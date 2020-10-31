% Generate 2 sin waves and plot the FFT
% Filter signal using FIR filter

% Close all figures
close all

% Sampling frequency
fe = 200;      
t = (0:1/fe:1-1/fe);

% Sinus
f0 = 5;
f1 = 50;
sinus1 = sin(2*pi*f0*t);
sinus2 = 0.5*sin(2*pi*f1*t);

% N, number of samples
N = length(sinus1);      
f = (-1/2*N+1:1/2*N);

% Add noise
sigma = 0.25;
mean = 1;
noise = mean + sigma * randn(1,N);

signal = sinus1 + sinus2 + noise;

% fftshift to center spectrum on 0
y = fftshift(abs(fft(signal)));

% FIR
filter = [-0.037841 -0.043247 -0.031183 0 0.046774 0.10091 0.15137 0.1871 0.2 0.1871 0.15137 0.10091 0.046774 0 -0.031183 -0.043247 -0.037841];

start = length(filter);
filtered_output = zeros(1,N-start);
original_output = zeros(1,N-start);

for i = start:N
   yout = 0;
   for k = 1:start
       yout = yout + filter(1,k) * signal(1, i - k + 1);
       z = 1 * signal(1, i - k + 1);
   end
   filtered_output(i) = yout;
   original_output(i) = z;
end

% Output spectrum
spectrum_filtered = fftshift(abs(fft(filtered_output)));

% Display results


subplot(2,2,1)
plot(original_output);
title('Input signal')

subplot(2,2,2)
plot(filtered_output);
title('Filtered signal')

subplot(2,2,3)
plot(f,y);
title('Input spectrum')

subplot(2,2,4)
plot(f,spectrum_filtered);
title('Output spectrum')

sgtitle('FIR filter')

% Display Bode diagram
figure;
num = filter;
den = [ 1 ];
H = tf(num, den, 1/fe);
bode(H)