% Defining A Source Using An Array Transducer Example
% 
% This example provides a demonstration of using the kWaveArray class to
% define an array transducer with three arc-shaped elements without
% staircasing errors.
%
% For a more detailed discussion of this example and the underlying
% techniques, see E. S. Wise, B. T. Cox, J. Jaros, & B. E. Treeby (2019).
% Representing arbitrary acoustic source and sensor distributions in
% Fourier collocation methods. The Journal of the Acoustical Society of
% America, 146(1), 278-288. https://doi.org/10.1121/1.5116132.
%
% author: Bradley Treeby
% date: 4th September 2018
% last update: 2nd November 2022
%  
% This function is part of the k-Wave Toolbox (http://www.k-wave.org)
% Copyright (C) 2018-2022 Bradley Treeby

% This file is part of k-Wave. k-Wave is free software: you can
% redistribute it and/or modify it under the terms of the GNU Lesser
% General Public License as published by the Free Software Foundation,
% either version 3 of the License, or (at your option) any later version.
% 
% k-Wave is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
% FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
% more details. 
% 
% You should have received a copy of the GNU Lesser General Public License
% along with k-Wave. If not, see <http://www.gnu.org/licenses/>. 

clearvars;



% =========================================================================
% DEFINE GRID PROPERTIES
% =========================================================================

% grid properties
Nx = 256; %256
dx = 0.5e-3;
Ny = 512; %256
dy = 0.5e-3;
kgrid = kWaveGrid(Nx, dx, Ny, dy);


% medium properties
medium.sound_speed = 343;
medium.density = 1.18;          % [kg/m^3]
% medium.alpha_coeff = 0.75;  % [dB/(MHz^y cm)]
% medium.alpha_power = 1.5;
medium.alpha_coeff = 0.003;      % [dB/(MHz^y cm)]
medium.alpha_power = 0.1;

% time array
t_end = 60e-5;                  % [s]
kgrid.makeTime(medium.sound_speed, [], t_end);


% =========================================================================
% DEFINE KWAVEARRAY
% =========================================================================

% create empty array
karray1 = kWaveArray;
karray2 = kWaveArray;


% karray.addLineElement([0,5e-3],[0,15e-3]);
% karray.addLineElement([10e-3,20e-3],[10e-3,30e-3]);
% karray.addLineElement([20e-3,35e-3],[20e-3,45e-3]);


N=17;
w=10e-3;dif=5e-3;d=dif;
ofs = 0.5e-3;
for i=1:N
%     karray.addLineElement([w*i,-N*w/2+((w+dif)*(i-1))], [w*i,-N*w/2+((w+dif)*(i-1))+w]);
%     karray.addLineElement([0,-N*w/2+((w+dif)*(i-1))-(Ny*dy)/2+(N)*(w)-dif], [0,-N*w/2+((w+dif)*(i-1))+w-(Ny*dy)/2+(N)*(w)-dif]);
    karray1.addLineElement([-Nx/2*dx+(ofs),d*i+(i-1)*w-(Ny*dy)/2], [-Nx/2*dx+(ofs),d*i+i*w-(Ny*dy)/2]);
    karray1.addLineElement([Nx/2*dx-(ofs),d*i+(i-1)*w-(Ny*dy)/2], [Nx/2*dx-(ofs),d*i+i*w-(Ny*dy)/2]);
%     karray2.addLineElement([30e-3,d*i+(i-1)*w-(Ny*dy)/2], [30e-3,d*i+i*w-(Ny*dy)/2]);
end



% =========================================================================
% SIMULATION
% =========================================================================

% assign binary mask from karray to the source mask
source1.p_mask = karray1.getArrayBinaryMask(kgrid);
% source2.p_mask = karray2.getArrayBinaryMask(kgrid);

source_freq = 40e3; % [Hz]
source_mag = 0.5; % [Pa]

% combine source signals into one array
sig = source_mag * sin(2 * pi * source_freq * kgrid.t_array + pi);
source1_signal = zeros(3, length(sig));
source2_signal = zeros(3, length(sig));

for i=1:2*N
    source1_signal(i,:) = sig;
    j=i+1;
    source1_signal(j,:) = sig;
%     source2_signal(i,:) = sig;
end

% get distributed source signals (this automatically returns a weighted
% source signal for each grid point that forms part of the source)
source1.p = karray1.getDistributedSourceSignal(kgrid, source1_signal);
% source2.p = karray2.getDistributedSourceSignal(kgrid, source2_signal);

[m,n]=size(source1.p);
for i=1:m
    source1.p(i,:) = filterTimeSeries(kgrid, medium, source1.p(i,:));
end

source.p = source1.p;
source.p_mask = source1.p_mask;
% source.p = source1.p + source2.p;
% source.p_mask = source1.p_mask + source2.p_mask;

display_mask = source.p_mask;

% create a sensor mask covering the entire computational domain using the
% opposing corners of a rectangle
sensor.mask = [1, 1, Nx, Ny].';

% set the record mode capture the final wave-field and the statistics at
% each sensor point 
sensor.record = {'p_final', 'p_max', 'p_rms'};

input_args = {'DisplayMask', display_mask, 'PMLInside', false, 'PlotPML', false};

% run k-Wave simulation (no sensor is used for this example)
sensor_data = kspaceFirstOrder2D(kgrid, medium, source, sensor,input_args{:});

% =========================================================================
% VISUALISATION
% =========================================================================


% add the source mask onto the recorded wave-field
sensor_data.p_final(source.p_mask ~= 0) = 1;
sensor_data.p_max(source.p_mask ~= 0) = 1;
sensor_data.p_rms(source.p_mask ~= 0) = 1;

% plot the final wave-field
figure;
subplot(1, 3, 1);
imagesc(kgrid.y_vec * 1e3, kgrid.x_vec * 1e3, sensor_data.p_final, [-1 1]);
colormap(getColorMap);
ylabel('x-position [mm]');
xlabel('y-position [mm]');
axis image;
title('Final Wave Field');

% plot the maximum recorded pressure
subplot(1, 3, 2);
imagesc(kgrid.y_vec * 1e3, kgrid.x_vec * 1e3, sensor_data.p_max, [-1 1]);
colormap(getColorMap);
ylabel('x-position [mm]');
xlabel('y-position [mm]');
axis image;
title('Maximum Pressure');

% plot the rms recorded pressure
subplot(1, 3, 3);
imagesc(kgrid.y_vec * 1e3, kgrid.x_vec * 1e3, sensor_data.p_rms, [-1 1]);
colormap(getColorMap);
ylabel('x-position [mm]');
xlabel('y-position [mm]');
axis image;
title('RMS Pressure');
scaleFig(2, 1);

