function [ fwd_kin_result ] = Run_Plot_Fwd_Kinematics(theta1, theta2, theta3, theta4)
    % Denavit-Hartenberg parameters
    global r2;
    global r3;
    global r4;
    global d1;
    global alpha1;

    % Denavit-Hartenberg parameters matrix reference:
    % [theta1  theta2  theta3  theta4 ;     % theta
    %  d1      d2      d3      d4;          % d
    %  r1      r2      r3      r4;          % r
    %  alpha1  alpha2  alpha3  alpha4 ]     % alpha

    % If you want to keep the last link (gripper) always horizontal:
%     theta4 = 360 - (theta2 + theta3); 

    % If you want to keep the last link (gripper) in zero degrees, 
    % so it is co-lineal with the previous link:
%     theta4 = 0; 

    % Denavit-Hartenberg parameters matrix
    dh_params = [theta1      theta2    theta3    theta4;    % theta
                 d1          0         0         0;         % d
                 0           r2        r3        r4;        % r
                 alpha1      0         0         0];        % alpha

    % Compute forward kinematics
    fwd_kin_result = Calc_Fwd_Kinematics(dh_params);

    % Get x, y, z coordinates for each link
    Q = Get_Link_Coordinates(fwd_kin_result);

    % Plot links in the GUI
    plot3(Q(1,:), Q(2,:), Q(3,:), '-o', 'LineWidth', 8, 'MarkerSize', 10, ...
          'MarkerFaceColor', [0.0, 0.5, 0.5], 'Color', 'Blue');
    grid on;

    % Plot coordinates of the end of the last link (gripper)
    text(Q(1, 5)+1, Q(2 ,5)+1, Q(3, 5)+1, ['  (', num2str(Q(1,5),3), ...
                                     ', ', num2str(Q(2,5),3), ...
                                     ', ', num2str(Q(3,5),3), ')']);

    % Set the plot title
%     title('4 DOF Robotic Arm Control')

    % Label the axes
    xlabel('X', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'blue');
    ylabel('Y', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'blue');
    zlabel('Z', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'blue');

    % Max and min for each cartesian axis
    maxX = r2 + r3 + r4; % Add link lenghts from link 2 to the last
    maxY = maxX; % Same for Y
    maxZ = d1 + r2 + r3 + r4; % Add link lenghts, including d1
    minZ = 0; % Zero in 'Z' is at the robotic arm's base
    
    % Set max and min for each axis in the plot
    axis([-maxX maxX -maxY maxY minZ maxZ]);
    
    h = rotate3d; % Get a handle to the 3D rotation mode of the axes
    h.Enable = 'on'; % Enable 3D rotation
    h.ActionPostCallback = @rotate3d_postcallback; % Configure postcallback
    
    % Save forward kinematics result in 'base'workspace
    assignin('base', 'fwd_kin_result', fwd_kin_result);
    
    % Get last 3D perspective set by the user in the GUI and apply it to
    % current view
    view(evalin('base', 'coord_3d_perspective'));

    % Delay to control the robotic arm's animation velocity whe doing the
    % inverse kinematics
    pause(0.1); 
end

% Callback function run when the user rotates the cartesian axes
function rotate3d_postcallback(obj, event_obj)

cartesian_axes = get(gca); % Get a handle to the cartesian axes

% Save current 3D perspective to 'base' workspace
assignin('base', 'coord_3d_perspective', cartesian_axes.View);
end
