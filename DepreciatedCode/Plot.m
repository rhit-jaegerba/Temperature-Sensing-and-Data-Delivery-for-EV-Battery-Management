% MATLAB script to plot data from Team_9_01212025_173807.csv

% Read data with original column headers preserved
filename = 'Team_9_01212025_173807.csv';
data = readtable(filename, 'VariableNamingRule', 'preserve');

% Convert Time to datetime
time = datetime(data.Time, 'InputFormat', 'hh:mm:ss a');

% Extract columns for plotting
input1 = data.('Univ Input 1 Module 1 (Deg C)');
heatPower = data.('Temperature Heat Power (%)');
pvTemperature = data.('Temperature PV (Deg C)');
setPoint = data.('Temperature Set Point (Deg C)');

% Plot the data
figure;
hold on;
plot(time, input1, '-o', 'DisplayName', 'Input 1 (Deg C)');
%plot(time, heatPower, '-x', 'DisplayName', 'Heat Power (%)');
plot(time, pvTemperature, '-s', 'DisplayName', 'PV Temperature (Deg C)');
plot(time, setPoint, '-d', 'DisplayName', 'Set Point (Deg C)');

% Customize the plot
xlabel('Time');
ylabel('Values');
title('Temperature and Heat Power Data');
legend('Location', 'best');
grid on;
hold off;
