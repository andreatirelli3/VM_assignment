function plotter(t, x, position, delta)
% len = length(t);

disp(length(delta));
disp(length(t));
disp(length(x(3,1:end)));
figure;

%% Global frame position
subplot(3,1,1);
plot(position(1,1:end),position(2,1:end), 'r-', 'LineWidth', 1.5)
title('Vehicle position in th global frame');
xlabel('X[m]');
ylabel('Y[m]');
grid on;

subplot(3,1,2);
plot(t, delta(1,1:end), 'g-', 'LineWidth', 1.5);
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle (°)');
grid on;

subplot(3,1,3);
plot(t, x(3,1:end), 'b-', 'LineWidth', 1.5);
title('Heading');
xlabel('Time (s)');
ylabel('Heading Angle (°)');
grid on;

end