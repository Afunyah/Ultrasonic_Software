% toneBurst(5e6, 40e3, 2, 'Plot', true);
% define sampling parameters
f = 40e3;
T = 1/f;
Fs = 100e6;
dt = 1/Fs;
t_array = 0:dt:10*T;

% define amplitude and phase
% amp = getWin(9, 'Gaussian');
% phase = linspace(0, 2*pi, 9).';

% create signals and plot
cw_signal = createCWSignals(t_array, f, 20, 0);
stackedPlot(cw_signal);
