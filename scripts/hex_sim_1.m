
% author: Bradley Treeby
% date: 20th January 2010
% last update: 4th May 2017
%  
% This function is part of the k-Wave Toolbox (http://www.k-wave.org)
% Copyright (C) 2010-2017 Bradley Treeby

% This file is part of k-Wave. k-Wave is free software: you can
% redistribute it and/or modify it under the terms of the GNU Lesser
% General Public License as published by the Free Software Foundation,
% either version 3 of the License, or (at your option) any later version.

%-------
% Modified by Marcel Afunyah for academic use
% October 2023
%-------

clearvars; 




% =========================================================================
% SIMULATION
% =========================================================================

% create the computational grid
% Nx = 64;            % number of grid points in the x direction
% Ny = 64;            % number of grid points in the y direction
% Nz = 64;            % number of grid points in the z direction
Nx = 64;            % number of grid points in the x direction
Ny = 64;            % number of grid points in the y direction
Nz = 64;            % number of grid points in the z direction
dx = 0.1e-3;        % grid point spacing in the x direction [m]
dy = 0.1e-3;        % grid point spacing in the y direction [m]
dz = 0.1e-3;        % grid point spacing in the z direction [m]
kgrid = kWaveGrid(Nx, dx, Ny, dy, Nz, dz);

% define the properties of the propagation medium
% medium.sound_speed = 1500 * ones(Nx, Ny, Nz);	% [m/s]
% medium.density = 1000 * ones(Nx, Ny, Nz);       % [kg/m^3]

% define the properties of the propagation medium
medium.sound_speed = 1500 * ones(Nx, Ny, Nz);	% [m/s]
medium.density = 1000 * ones(Nx, Ny, Nz);       % [kg/m^3]

% medium.sound_speed = 1500 * ones(Nx, Ny, Nz);	% [m/s]
% medium.sound_speed(1:Nx/2, :, :) = 1800;        % [m/s]
% medium.density = 1000 * ones(Nx, Ny, Nz);       % [kg/m^3]
% medium.density(:, Ny/4:end, :) = 1200;          % [kg/m^3]

% t_end = 60e-7;                  % [s]
% kgrid.makeTime(medium.sound_speed, [], t_end);

kgrid.makeTime(medium.sound_speed);




% =========================================================================
% DEFINE KWAVEARRAY
% =========================================================================

% create empty array
karray1 = kWaveArray('BLITolerance', 0.05, 'UpsamplingRate', 10);

element_num     = 3;       % number of elements
element_width   = 1e-3;     % width [m]
element_length  = 1e-3;    % elevation height [m]
element_pitch   = 1.1e-3;     % pitch [m]
N=element_num;
w=10e-3;dif=5e-3;d=dif;
ofs = 0.5e-3;
% for i=1:N
% %     karray.addLineElement([w*i,-N*w/2+((w+dif)*(i-1))], [w*i,-N*w/2+((w+dif)*(i-1))+w]);
% %     karray.addLineElement([0,-N*w/2+((w+dif)*(i-1))-(Ny*dy)/2+(N)*(w)-dif], [0,-N*w/2+((w+dif)*(i-1))+w-(Ny*dy)/2+(N)*(w)-dif]);
%     karray1.addLineElement([-Nx/2*dx+(ofs),d*i+(i-1)*w-(Ny*dy)/2], [-Nx/2*dx+(ofs),d*i+i*w-(Ny*dy)/2]);
%     karray1.addLineElement([Nx/2*dx-(ofs),d*i+(i-1)*w-(Ny*dy)/2], [Nx/2*dx-(ofs),d*i+i*w-(Ny*dy)/2]);
% %     karray2.addLineElement([30e-3,d*i+(i-1)*w-(Ny*dy)/2], [30e-3,d*i+i*w-(Ny*dy)/2]);
% end

for ind = 1:element_num
    
    % set element y position
%     x_pos = 0 - (element_num * element_pitch / 2 - element_pitch / 2) + (ind - 1) * element_pitch;
    x_pos = 0 - (element_num * element_pitch / 1 - element_pitch / 1) + (ind - 1) * element_pitch*2;
    % add element (see note in header) kgrid.z_vec(1)
%     karray1.addRectElement([x_pos, 0, -1.6e-3], element_width, element_length, [0,0,0]);
    karray1.addDiscElement([x_pos, 0, -5e-3], element_width,[x_pos,0,0]);
end



% % define a square source element
% source_radius = 5;  % [grid points]
% source.p_mask = zeros(Nx, Ny, Nz);
% source.p_mask(Nx/4, Ny/2 - source_radius:Ny/2 + source_radius, Nz/2 - source_radius:Nz/2 + source_radius) = 1;
% 
% % define a time varying sinusoidal source
% source_freq = 2e6;  % [Hz]
% source_mag = 1;     % [Pa]
% source.p = source_mag * sin(2 * pi * source_freq * kgrid.t_array);
% 
% % filter the source to remove high frequencies not supported by the grid
% source.p = filterTimeSeries(kgrid, medium, source.p);

% assign binary mask from karray to the source mask

source1.p_mask = karray1.getArrayBinaryMask(kgrid);

source_freq = 40e3; % [Hz]
source_mag = 2; % [Pa]
source_phs = 0;
source_freq = 2e6;  % [Hz]
source_mag = 1;     % [Pa]

% combine source signals into one array
sig = source_mag * sin(2 * pi * source_freq * kgrid.t_array + source_phs);
source1_signal = zeros(3, length(sig));

% for i=1:2*N
%     source1_signal(i,:) = sig;
%     j=i+1;
%     source1_signal(j,:) = sig;
% end

for i=1:N
    source1_signal(i,:) = sig;
end


% get distributed source signals (this automatically returns a weighted
% source signal for each grid point that forms part of the source)
source1.p = karray1.getDistributedSourceSignal(kgrid, source1_signal);

[m,n]=size(source1.p);
for i=1:m
    source1.p(i,:) = filterTimeSeries(kgrid, medium, source1.p(i,:));
end

source.p = source1.p;
source.p_mask = source1.p_mask;

display_mask = source.p_mask;


% define a series of Cartesian points to collect the data
y = (-20:2:20) * dy;            % [m]
z = (-20:2:20) * dz;            % [m]
x = 20 * dx * ones(size(z));    % [m]
sensor.mask = [x; y; z];

% define the field parameters to record
sensor.record = {'p', 'p_final', 'p_max'};

% input arguments
% input_args = {'DisplayMask', source.p_mask, 'DataCast', 'single', 'CartInterp', 'nearest','PMLInside', false, 'PlotPML', false};
input_args = {'DisplayMask', display_mask, 'DataCast', 'single', 'CartInterp', 'nearest','PMLInside', true, 'PlotPML', false};


% run the simulation
sensor_data = kspaceFirstOrder3D(kgrid, medium, source, sensor, input_args{:});
% 
% % =========================================================================
% % VISUALISATION
% % =========================================================================
% 
% % view final pressure field slice by slice
% flyThrough(sensor_data.p_final);
% 
% % plot the position of the source and sensor
fig=figure;
voxelPlot_c(double(source.p_mask | cart2grid(kgrid, sensor.mask)), fig, 'AxisTight', false, 'Color', [0 0 1]);
% voxelPlot_c(makeBall(20, 20, 20, 10, 10, 10, 4), fig, 'AxisTight', true, 'Color', [1 0 0]);
% 

% add the source mask to the pressure field
p_final(source.p_mask ~= 0) = 1;

% extract a suitable axis scaling factor
[~, scale, prefix] = scaleSI(max([kgrid.x_vec, kgrid.y_vec])); 

% plot the final pressure field in the x-y plane
figure;
imagesc(kgrid.y_vec * scale, kgrid.x_vec * scale, squeeze(sensor_data.p_final(:, :, kgrid.Nz/2)), [-1, 1]);
colormap(getColorMap);
xlabel(['y-position [' prefix 'm]']);
ylabel(['x-position [' prefix 'm]']);
axis image;
colorbar;

% plot the simulated sensor data
figure;
imagesc(sensor_data.p, [-1, 1]);
colormap(getColorMap);
ylabel('Sensor Position');
xlabel('Time Step');
colorbar;

% plot the pressure field 
figure;
imagesc(scale * kgrid.z_vec, scale * kgrid.x_vec, squeeze(sensor_data.p_final(:, :, kgrid.Nz/2)));

xlabel('Axial Position [mm]');
ylabel('Lateral Position [mm]');
axis image;
title('Pressure Field');

