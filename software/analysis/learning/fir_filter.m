% First part: generate 2 sin waves and plot the FFT

% Sampling frequency
fe = 200;      
t = (0:1/fe:1-1/fe);
% N, number of samples
N = length(sinus);      
f = (-1/2*N+1:1/2*N);

% Sinus
f0 = 20;
f1 = 50;
sinus1 = sin(2*pi*f0*t);
sinus2 = 0.5*sin(2*pi*f1*t);

% Add noise
sigma = 0.25;
moy = 1;
noise = moy + sigma*randn(1,N);

signal = sinus1 + sinus2 + noise;    

% fftshift to center spectrum on 0
y = fftshift(abs(fft(signal)));

plot(f,y)
f = xlabel('Frequency (Hz)')
y = ylabel('Amplitude')