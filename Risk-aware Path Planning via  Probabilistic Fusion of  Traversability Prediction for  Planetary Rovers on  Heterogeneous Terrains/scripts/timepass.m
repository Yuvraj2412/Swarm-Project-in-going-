% Parameters
A   = 1;      % Pulse amplitude
k   = 1;      % Static sensitivity
tau = 1;      % Time constant
T   = 2;      % Pulse duration

% Time vectors
t1 = linspace(0, T, 500);    % 0 < t < T
t2 = linspace(T, 6, 500);    % t > T

% Response equations
y1 = k*A*(1 - exp(-t1/tau));                  % Pulse ON
y2 = k*A*(exp(T/tau) - 1).*exp(-t2/tau);      % Pulse OFF

% Plot
figure
plot(t1, y1, 'b', 'LineWidth', 2)
hold on
plot(t2, y2, 'r', 'LineWidth', 2)
xline(T, '--k', 'LineWidth', 1.5)

% Labels and formatting
xlabel('Time (s)')
ylabel('Output y(t)')
title('Response of First-Order Instrument to a Pulse Input')
legend('0 < t < T (Pulse ON)', 't > T (Pulse OFF)', 'Pulse ends at T')
grid on
