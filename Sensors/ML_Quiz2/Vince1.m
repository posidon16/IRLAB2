% Define system matrices A and B
A = [1, 2; 2, 1];
B = [0; 1];

% Define initial condition x(0)
% x0 = [x_0; y_0]; % Replace x_0 and y_0 with your specific values
x0 = [0;0];

% Define control input u(0) if available
u0 = u_0; % Replace u_0 with your specific value if available

% Calculate the state at k = 1
x1 = A * x0 + B * u0; % If u0 is not specified, leave it as [0; 0]

% Display the result
disp('State at k = 1:');
disp(x1);
