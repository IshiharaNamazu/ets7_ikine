% Example data for pos_e_history
% Assuming pos_e_history is a matrix with three columns representing x, y, and z coordinates

% Extracting x, y, z coordinates
x = pos_e_history(1,:);
y = pos_e_history(2,:);
z = pos_e_history(3,:);

% Creating the 3D plot
figure;
plot3(x, y, z, 'LineWidth', 2);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Plot of pos_e_history');
xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);
axis equal;
grid on; % Optional: Add grid for better visualization
