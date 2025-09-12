filepath = "C:\Users\jonat\Downloads\PositionTest_12_09_2025_15_27.csv";
arr = readtable(filepath);

x = arr.x;
y = arr.y;
z = arr.z;
t=arr.t;

%need to get t

%plots lines between the points
plot3(x, y, z, 'w', 'LineWidth', 1.5);  
hold on

% Then, plot the scatter points colored by elapsed time
scatter3(x, y, z, 50, t, 'filled');

hold off
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D Scatter Plot with Lines Connecting Points')
colorbar
colormap jet
view(45, 45)
