[audio1, sampleRate1] = audioread("Recording1\file_stereo.wav");
[audio2, sampleRate2] = audioread("Recording2\file_stereo.wav");

sig1 = audio1(:,1);
sig2 = audio1(:,2);
sig3 = audio2(:,1);
sig4 = audio2(:,2);

c = 343;
% Microphone positions
mic = [0, 0;      % Mic 1
       0.0, 0.5;    % Mic 4 
       0.8, 0.5;    % Mic 3
       0.8, 0.0]; % Mic 4


fc_low = 300;
fc_high = 3000;

% Define the filter order (higher order results in steeper roll-off)
filterOrder = 4;

% Design a Butterworth low-pass filter
[b, a] = butter(filterOrder, [fc_low, fc_high] / (48000/2), 'bandpass');

% Apply the filter to your signal
sig1 = filter(b, a, sig1);
sig2 = filter(b, a, sig2);
sig3 = filter(b, a, sig3);
sig4 = filter(b, a, sig4);

sig1(1:0.3*48000) = 0;
sig2(1:0.3*48000) = 0;
sig3(1:0.3*48000) = 0;
sig4(1:0.3*48000) = 0;

% Calculate the duration of the audio in seconds
durationInSeconds = numel(sig1) / sampleRate1;
time = linspace(0, durationInSeconds, numel(sig1));

cal_delay = 15*sampleRate1;

%Calibration
cal1 = sig1(1:cal_delay);
cal2 = sig2(1:cal_delay);
cal3 = sig3(1:cal_delay);
cal4 = sig4(1:cal_delay);
%After calibration
sig1 = sig1(cal_delay:end);
sig2 = sig2(cal_delay:end);
sig3 = sig3(cal_delay:end);
sig4 = sig4(cal_delay:end);

durationInSeconds = numel(cal1) / sampleRate1;
time = linspace(0, durationInSeconds, numel(cal1));

figure;
plot(time, cal1, 'DisplayName', 'Audio 1 After Alignment'); hold on;
plot(time, cal3, 'DisplayName', 'Audio 2 After Alignment'); hold off;
title('Signals After Alignment');
legend('show');

[crossCorr, lags] = xcorr(cal1, cal3);
[~, maxIndex] = max(abs(crossCorr));
sampleDelay = lags(maxIndex);  % This is the delay in number of samples

if sampleDelay > 0
    sig1 = sig1(sampleDelay+1:end); % Trim audio1 to synchronize
    sig2 = sig2(sampleDelay+1:end);
elseif sampleDelay < 0
    sig3 = sig3(abs(sampleDelay)+1:end);
    sig4 = sig4(abs(sampleDelay)+1:end);
end

% Ensure both audio signals are of equal length after trimming
maxLength = max(length(sig1), length(sig3));
sig1 = [sig1;zeros(maxLength-length(sig1),1)];
sig2 =[sig2;zeros(maxLength-length(sig2),1)];
sig3 = [sig3;zeros(maxLength-length(sig3),1)];
sig4 =[sig4;zeros(maxLength-length(sig4),1)];

% figure;
% plot(time, audio1, 'DisplayName', 'Audio 1 After Alignment'); hold on;
% plot(time, audio2, 'DisplayName', 'Audio 2 After Alignment'); hold off;
% title('Signals After Chop');
% legend('show');

noisy_signals = [sig1, sig2, sig3, sig4];
time_diffs = zeros(1,3);
for i = 2:4
    [cross_corr, lags] = xcorr(noisy_signals(:,1), noisy_signals(:,i));
    [~, idx] = max(cross_corr);
    time_diffs(i-1) = lags(idx) / sampleRate1;
end

% Generate symbolic hyperbolic equations
syms x y

hyperbolae = cell(1, 3);
for i = 2:4
    equation = sqrt((x - mic(i,1))^2 + (y - mic(i,2))^2) - sqrt((x-mic(1,1))^2 + (y-mic(1,2))^2) + c * time_diffs(i-1);
    hyperbolae{i-1} = equation;  
end

% Generate a grid of points to evaluate the hyperbolic equation
x_range = 0:0.001:0.8;
y_range = 0:0.001:0.5;
[X, Y] = meshgrid(x_range, y_range);

% Plot the hyperbolas
figure; hold on;
colors = ['r', 'g', 'b'];
for i = 2:4
    Z = c*time_diffs(i-1) + sqrt((X-mic(i,1)).^2 + (Y-mic(i,2)).^2) - sqrt((X-mic(1,1)).^2 + (Y-mic(1,2)).^2);
    contour(X, Y, Z, [0 0], colors(i-1));
end

solutions1 = solve([hyperbolae{1}, hyperbolae{2}], [x, y]);
solutions2 = solve([hyperbolae{2}, hyperbolae{3}], [x, y]);
solutions3 = solve([hyperbolae{1}, hyperbolae{3}], [x, y]);

% Extract x and y values of the intersections
x_intersections1 = double(solutions1.x);
y_intersections1 = double(solutions1.y);
x_intersections2 = double(solutions2.x);
y_intersections2 = double(solutions2.y);
x_intersections3 = double(solutions3.x);
y_intersections3 = double(solutions3.y);


% Getting the vertices of the triangle
A = [x_intersections1, y_intersections1];
B = [x_intersections2, y_intersections2];
C = [x_intersections3, y_intersections3];

% Compute centroid
centroid_x = (A(1) + B(1) + C(1)) / 3;
centroid_y = (A(2) + B(2) + C(2)) / 3;

centroid = [centroid_x, centroid_y];

% Display the centroid
disp('Centroid of the triangle:');
disp(centroid);

% Plot the microphones' positions
plot(mic(:,1), mic(:,2), 'k^', 'MarkerSize', 12, 'DisplayName', 'Microphones');

% Plot the centroid of the triangle
plot(centroid_x, centroid_y, 'o', 'Color', [0.6 0.2 0], 'MarkerSize', 10, 'DisplayName', 'Centroid');

% Some graphical adjustments
legend('show'); % To show a legend on the plot
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Hyperbolae Intersections, Source, and Centroid');
grid on;
axis([0 0.8 0 0.5]); % Setting the axis limits