function varargout = Robotic_Arm_GUI(varargin)
% Robotic_Arm_GUI MATLAB code for Robotic_Arm_GUI.fig
%      Robotic_Arm_GUI, by itself, creates a new Robotic_Arm_GUI or raises the existing
%      singleton*.
%
%      H = Robotic_Arm_GUI returns the handle to a new Robotic_Arm_GUI or the handle to
%      the existing singleton*.
%
%      Robotic_Arm_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in Robotic_Arm_GUI.M with the given input arguments.
%
%      Robotic_Arm_GUI('Property','Value',...) creates a new Robotic_Arm_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Robotic_Arm_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Robotic_Arm_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Robotic_Arm_GUI

% Last Modified by GUIDE v2.5 09-Sep-2019 19:16:06

% Begin initialization code - DO NOT EDIT

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Robotic_Arm_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Robotic_Arm_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Robotic_Arm_GUI is made visible.
function Robotic_Arm_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Robotic_Arm_GUI (see VARARGIN)

% Choose default command line output for Robotic_Arm_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Robotic_Arm_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure_gui);

% Open serial port
disp('Opening serial port...');
global serial_port; % Global variable for referencing the serial port
serial_port = serial('COM50'); % Change this to your Arduino's serial port name
set(serial_port, 'BaudRate', 9600);
fopen(serial_port);
disp('Serial port is open, waiting for the Arduino...');
pause(5); % To let the arduino get ready
disp('Ready to send commands!');

% Denavit-Hartenberg Constants
global r2;
global r3;
global r4;
global d1;
global alpha1;

% Link sizes
r2 = 10.5; % Link 2 size in cm
r3 = 10;   % Link 3 size in cm
r4 = 15;   % Link 4 size in cm (gripper)
d1 = 11.5; % Link 1 size in cm
alpha1 = 90; % Angle between 2nd. joint's 'z' axe and 1st. joint's 'z' axe

% Max and min angle values for each motor
% According to configuration space defined for each joint
global motor1_max;
global motor1_min;
global servo2_max;
global servo2_min;
global servo3_max;
global servo3_min;
global servo4_max;
global servo4_min;
global servo_grip_open;
global servo_grip_close;

% Limits of the configuration space for each joint
motor1_max = 90;
motor1_min = -90;
servo2_max = 155;
servo2_min = -25;
servo3_max = 45;
servo3_min = -135;
servo4_max = 90;
servo4_min = -90;
servo_grip_open = 50; % Max angle for open gripper
servo_grip_close = 90;   % Min angle for closed gripper

% Initial angles for each joint that define the start pose
theta_init = [0; 90; -90; -90];

% Set initial angles in GUI's angle value labels
set(handles.txt_motor1_angle, 'String', num2str(theta_init(1)));
set(handles.txt_servo2_angle, 'String', num2str(theta_init(2)));
set(handles.txt_servo3_angle, 'String', num2str(theta_init(3)));
set(handles.txt_servo4_angle, 'String', num2str(theta_init(4)));

% Set initial angles in GUI's angle sliders
set(handles.slider_motor1_angle, 'Value', theta_init(1));
set(handles.slider_servo2_angle, 'Value', theta_init(2));
set(handles.slider_servo3_angle, 'Value', theta_init(3));
set(handles.slider_servo4_angle, 'Value', theta_init(4));

% Save theta_init to variable 'theta_init' in workspace 'base'
assignin('base', 'theta_init', theta_init);

% Save desired initial 3D perspective for cartesian coordinates to variable
% 'coord_3d_perspective' in 'base' workspace
assignin('base', 'coord_3d_perspective', [-37 30]);

% Run forward kinematics calculation for initial angles and plot 
% the resulting pose in the GUI
% This function returns a matrix containing the coordinates of the ends of 
% every link in the robotic arm
Run_Plot_Fwd_Kinematics(theta_init(1, 1), theta_init(2, 1), theta_init(3, 1), theta_init(4, 1));

% Initialization code for inverse kinematics
% Read last pose calculated using forward kinematics from 'base' workspace
% See the 'Run_Plot_Fwd_Kinematics' file to see
fwd_kin_result = evalin('base', 'fwd_kin_result');

% Set initial and final position for inverse kinematics as the same
inv_kin_init_pos = [fwd_kin_result(1, 16); fwd_kin_result(2, 16); fwd_kin_result(3, 16)];
inv_kin_final_pos = inv_kin_init_pos;

% Save both to 'base' workspace
assignin('base', 'inv_kin_init_pos', inv_kin_init_pos);
assignin('base', 'inv_kin_final_pos', inv_kin_final_pos);

% Save initial angles to 'base' workspace for inverse kinematics
assignin('base', 'theta_start_invkin', theta_init); 

% Set initial x, y, z coordinates in corresponding GUI labels and sliders
% This initial coordinates come from the first forward kinematics
% calculation above
set(handles.txt_coordx,'String', num2str(fwd_kin_result(1, 16)));
set(handles.txt_coordy,'String', num2str(fwd_kin_result(2, 16)));
set(handles.txt_coordz,'String', num2str(fwd_kin_result(3, 16)));
set(handles.slider_coordx, 'value', fwd_kin_result(1, 16));
set(handles.slider_coordy, 'value', fwd_kin_result(2, 16));
set(handles.slider_coordz, 'value', fwd_kin_result(3, 16));

% Set increment steps for GUI motor1 slider
set(handles.slider_motor1_angle, 'Max', motor1_max);
set(handles.slider_motor1_angle, 'Min', motor1_min);
range = motor1_max - motor1_min;
% Short and long steps for the slider
steps = [1/range, 10/range];
set(handles.slider_motor1_angle, 'SliderStep', steps);

% Set increment steps for GUI servo2 slider
set(handles.slider_servo2_angle, 'Max', servo2_max);
set(handles.slider_servo2_angle, 'Min', servo2_min);
range = servo2_max - servo2_min;
% Short and long steps for the slider
steps = [1/range, 10/range];
set(handles.slider_servo2_angle, 'SliderStep', steps);

% Set increment steps for GUI servo3 slider
set(handles.slider_servo3_angle, 'Max', servo3_max);
set(handles.slider_servo3_angle, 'Min', servo3_min);
range = servo3_max - servo3_min;
% Short and long steps for the slider
steps = [1/range, 10/range];
set(handles.slider_servo3_angle, 'SliderStep', steps);

% Set increment steps for GUI servo4 slider
set(handles.slider_servo4_angle, 'Max', servo4_max);
set(handles.slider_servo4_angle, 'Min', servo4_min);
range = servo4_max - servo4_min;
% Short and long steps for the slider
steps = [1/range, 10/range];
set(handles.slider_servo4_angle, 'SliderStep', steps);

% Set incrememt steps for GUI x, y coordinate sliders
% Max and min are the same for x, y
max_slider_val = r2 + r3 + r4; % The sum of all links lenght
set(handles.slider_coordx, 'Max', max_slider_val);
set(handles.slider_coordy, 'Max', max_slider_val);
min_slider_val = -max_slider_val;
set(handles.slider_coordx, 'Min', min_slider_val);
set(handles.slider_coordy, 'Min', min_slider_val);
range = max_slider_val - min_slider_val;
% Short and long steps for the sliders
steps = [1/range, 10/range];
set(handles.slider_coordx, 'SliderStep', steps);
set(handles.slider_coordy, 'SliderStep', steps);

% Set incrememt steps for GUI z coordinate slider
max_slider_val = d1 + r2 + r3 + r4; % Longitud
set(handles.slider_coordz, 'Max', max_slider_val);
min_slider_val = 0;
set(handles.slider_coordz, 'Min', min_slider_val);
range = max_slider_val - min_slider_val;
% Short and long steps for the slider
steps = [1/range, 10/range];
set(handles.slider_coordz, 'SliderStep', steps);


% --- Outputs from this function are returned to the command line.
function varargout = Robotic_Arm_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


varargout{1} = handles.output;


function slider_motor1_angle_Callback(hObject, eventdata, handles)
% Get modified angle for current motor and display it in the corresponding
% GUI's text label
angle1 = get(hObject, 'Value') ;
set(handles.txt_motor1_angle, 'String', num2str(angle1, 3) );

% Get angles from the rest of the motors
angle2 = str2num(get(handles.txt_servo2_angle, 'String'));
angle3 = str2num(get(handles.txt_servo3_angle, 'String'));
angle4 = str2num(get(handles.txt_servo4_angle, 'String'));

% Create angles array
angles = [angle1; angle2; angle3; angle4];
assignin('base', 'theta_start_invkin', angles); % Start angles for inverse kinematics

% Compute forward kinematics and plot resulting pose in the GUI
Run_Plot_Fwd_Kinematics(angle1, angle2, angle3, angle4);

% -- Send new joint angles via serial communications
% Print angle data frames to command window
disp('Motor 1 angle changed, sending new angles:');
fprintf(strcat('1$', num2str(round(angle1)), '&'));
fprintf(strcat('2$', num2str(round(angle2)), '&'));
fprintf(strcat('3$', num2str(round(angle3)), '&'));
fprintf(strcat('4$', num2str(round(angle4)), '&'));

% Send angles through the serial port
global serial_port; 
fprintf(serial_port, strcat('1$', num2str(round(angle1)), '&'));
fprintf(serial_port, strcat('2$', num2str(round(angle2)), '&'));
fprintf(serial_port, strcat('3$', num2str(round(angle3)), '&'));
fprintf(serial_port, strcat('4$', num2str(round(angle3)), '&'));

% Read the forward kinematics calculation result to update GUI's coordinate
% sliders and text labels
fwd_kin_result = evalin('base', 'fwd_kin_result');

% Set new values in corresponding sliders and text labels
set(handles.txt_coordx, 'String', num2str(fwd_kin_result(1, 16)));
set(handles.txt_coordy, 'String', num2str(fwd_kin_result(2, 16)));
set(handles.txt_coordz, 'String', num2str(fwd_kin_result(3, 16)));
set(handles.slider_coordx, 'value', fwd_kin_result(1, 16));
set(handles.slider_coordy, 'value', fwd_kin_result(2, 16));
set(handles.slider_coordz, 'value', fwd_kin_result(3, 16));

% Update new init position for next inverse kinematics calculation
inv_kin_init_pos = [fwd_kin_result(1, 16); 
                    fwd_kin_result(2, 16); 
                    fwd_kin_result(3, 16)];
assignin('base', 'inv_kin_init_pos', inv_kin_init_pos);
assignin('base','theta_start_invkin', angles); % Start angles for inverse kinematics


function slider_motor1_angle_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slider_servo2_angle_Callback(hObject, eventdata, handles)
% Get modified angle for current motor and display it in the corresponding
% GUI's text label
angle2 = get(hObject, 'Value') ;
set(handles.txt_servo2_angle, 'String', num2str(angle2, 3));

% Get angles from the rest of the motors
angle1 = str2num(get(handles.txt_motor1_angle, 'String'));
angle3 = str2num(get(handles.txt_servo3_angle, 'String'));
angle4 = str2num(get(handles.txt_servo4_angle, 'String'));

% Create angles array
angles = [angle1; angle2; angle3; angle4];

% Compute forward kinematics and plot resulting pose in the GUI
Run_Plot_Fwd_Kinematics(angle1, angle2, angle3, angle4);
view(evalin('base', 'coord_3d_perspective')); % Aplicar perspectiva 3D

% -- Send new joint angles via serial communications
% Print angle data frames to command window
disp('Servo 2 angle changed, sending new angles:');
fprintf(strcat('1$', num2str(round(angle1)), '&'));
fprintf(strcat('2$', num2str(round(angle2)), '&'));
fprintf(strcat('3$', num2str(round(angle3)), '&'));
fprintf(strcat('4$', num2str(round(angle4)), '&'));

% Send angles through the serial port
global serial_port;
fprintf(serial_port, strcat('1$', num2str(round(angle1)), '&'));
fprintf(serial_port, strcat('2$', num2str(round(angle2)), '&'));
fprintf(serial_port, strcat('3$', num2str(round(angle3)), '&'));
fprintf(serial_port, strcat('4$', num2str(round(angle4)), '&'));

% Read the forward kinematics calculation result to update GUI's coordinate
% sliders and text labels
fwd_kin_result = evalin('base', 'fwd_kin_result');

% Set new values in corresponding sliders and text labels
set(handles.txt_coordx, 'String', num2str(fwd_kin_result(1, 16)));
set(handles.txt_coordy, 'String', num2str(fwd_kin_result(2, 16)));
set(handles.txt_coordz, 'String', num2str(fwd_kin_result(3, 16)));
set(handles.slider_coordx, 'value', fwd_kin_result(1, 16));
set(handles.slider_coordy, 'value', fwd_kin_result(2, 16));
set(handles.slider_coordz, 'value', fwd_kin_result(3, 16));

% Update new init position for next inverse kinematics calculation
inv_kin_init_pos = [fwd_kin_result(1, 16); 
                    fwd_kin_result(2, 16); 
                    fwd_kin_result(3, 16)];
assignin('base','inv_kin_init_pos', inv_kin_init_pos);
assignin('base','theta_start_invkin', angles); % Start angles for inverse kinematics

function slider_servo2_angle_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slider_servo3_angle_Callback(hObject, eventdata, handles)
% Get modified angle for current motor and display it in the corresponding
% GUI's text label
angle3 = get(hObject, 'Value');
set(handles.txt_servo3_angle, 'String', num2str(angle3, 3));

% Get angles from the rest of the motors
angle1 = str2num(get(handles.txt_motor1_angle, 'String'));
angle2 = str2num(get(handles.txt_servo2_angle, 'String'));
angle4 = str2num(get(handles.txt_servo4_angle, 'String'));

% Create angles array
angles = [angle1; angle2; angle3; angle4];

% Compute forward kinematics and plot resulting pose in the GUI
Run_Plot_Fwd_Kinematics(angle1, angle2, angle3, angle4);
view(evalin('base', 'coord_3d_perspective'));

% -- Send new joint angles via serial communications
% Print angle data frames to command window
disp('Servo 3 angle changed, sending new angles:');
fprintf(strcat('1$', num2str(round(angle1)), '&'));
fprintf(strcat('2$', num2str(round(angle2)), '&'));
fprintf(strcat('3$', num2str(round(angle3)), '&'));
fprintf(strcat('4$', num2str(round(angle4)), '&'));

% Send angles through the serial port
global serial_port; 
fprintf(serial_port,strcat('1$', num2str(round(angle1)), '&'));
fprintf(serial_port,strcat('2$', num2str(round(angle2)), '&'));
fprintf(serial_port,strcat('3$', num2str(round(angle3)), '&'));
fprintf(serial_port,strcat('4$', num2str(round(angle4)), '&'));

% Read the forward kinematics calculation result to update GUI's coordinate
% sliders and text labels
fwd_kin_result = evalin('base', 'fwd_kin_result');

% Set new values in corresponding sliders and text labels
set(handles.txt_coordx, 'String', num2str(fwd_kin_result(1, 16)));
set(handles.txt_coordy, 'String', num2str(fwd_kin_result(2, 16)));
set(handles.txt_coordz, 'String', num2str(fwd_kin_result(3, 16)));
set(handles.slider_coordx, 'value', fwd_kin_result(1, 16));
set(handles.slider_coordy, 'value', fwd_kin_result(2, 16));
set(handles.slider_coordz, 'value', fwd_kin_result(3, 16));

% Update new init position for next inverse kinematics calculation
inv_kin_init_pos = [fwd_kin_result(1, 16); 
                    fwd_kin_result(2, 16); 
                    fwd_kin_result(3, 16)];
assignin('base', 'inv_kin_init_pos', inv_kin_init_pos);
assignin('base', 'theta_start_invkin', angles); % Start angles for inverse kinematics


function slider_servo3_angle_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes when user attempts to close figure_gui.
function figure_gui_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure_gui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Close serial port
global serial_port;
fclose(serial_port);
delete(serial_port);
clear serial_port;
disp('Serial port closed.');

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on slider movement.
function slider_coordx_Callback(hObject, eventdata, handles)
% hObject    handle to slider_coordx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get x, y, z coordinate values
coord_y = str2num(get(handles.txt_coordy, 'String'));
coord_z = str2num(get(handles.txt_coordz, 'String'));
coord_x = get(hObject, 'Value') ;

% Change value changed for this current slider in corresponding text label
set(handles.txt_coordx, 'String', num2str(coord_x, 3));

% Save new goal coordinates to calculate inverse kinematics once the GUI's
% activation button has been pressed.
coordinates = [coord_x; coord_y; coord_z];
assignin('base', 'inv_kin_final_pos', coordinates);


% --- Executes during object creation, after setting all properties.
function slider_coordx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_coordx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_coordy_Callback(hObject, eventdata, handles)
% hObject    handle to slider_coordy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get x, y, z coordinate values
coord_x = str2num(get(handles.txt_coordx, 'String'));
coord_z = str2num(get(handles.txt_coordz, 'String'));
coord_y = get(hObject, 'Value');

% Change value changed for this current slider in corresponding text label
set(handles.txt_coordy, 'String', num2str(coord_y,3));

% Save new goal coordinates to calculate inverse kinematics once the GUI's
% activation button has been pressed.
coordinates = [coord_x; coord_y; coord_z];
assignin('base', 'inv_kin_final_pos', coordinates);


% --- Executes during object creation, after setting all properties.
function slider_coordy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_coordy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_coordz_Callback(hObject, eventdata, handles)
% hObject    handle to slider_coordz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get x, y, z coordinate values
coord_x = str2num(get(handles.txt_coordx, 'String'));
coord_y = str2num(get(handles.txt_coordy, 'String'));
coord_z = get(hObject, 'Value');

% Change value changed for this current slider in corresponding text label
set(handles.txt_coordz, 'String', num2str(coord_z, 3));

% Save new goal coordinates to calculate inverse kinematics once the GUI's
% activation button has been pressed.
coordinates = [coord_x; coord_y; coord_z];
assignin('base', 'inv_kin_final_pos', coordinates);


% --- Executes during object creation, after setting all properties.
function slider_coordz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_coordz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in button_calc_invkin.
function button_calc_invkin_Callback(hObject, eventdata, handles)
% hObject    handle to button_calc_invkin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global serial_port;

% Get the start and goal positions for inverse kinematics
inv_kin_init_pos = evalin('base', 'inv_kin_init_pos');
inv_kin_final_pos = evalin('base', 'inv_kin_final_pos');

% Get the start angles for inverse kinematics
theta_start_invkin = evalin('base', 'theta_start_invkin'); 
theta1 = theta_start_invkin(1, 1); % Start angle for motor1
theta2 = theta_start_invkin(2, 1); % Start angle for servo2
theta3 = theta_start_invkin(3, 1); % Start angle for servo3
theta4 = theta_start_invkin(4, 1); % Start angle for servo4

% Save current angles as previous for inverse kinematics next iterations
theta1_prev = theta1;
theta2_prev = theta2;
theta3_prev = theta3;
theta4_prev = theta4;

% Verify if start and final position are different to run the inverse
% kinematics
if inv_kin_final_pos == inv_kin_init_pos
    % No need to run inverse kinematics
    disp('Please, choose a final position first.');
    assignin('base', 'invkin_calc_on', 0); % Deactivate inv. kin. calculation
else
    % Activate inv. kin. calculation
    assignin('base', 'invkin_calc_on', 1); 
    % Calculate position delta; delta = [dX; dY; dZ; EucXY; EucXYZ; Zang]
    delta = Calc_Pos_Delta(inv_kin_init_pos, inv_kin_final_pos);

    % While inverse kinematics calculation is on...
    while (evalin('base', 'invkin_calc_on') == 1)

        % Compute delta_theta using the pseudo-inverse Jacobian
        fwd_kin_result = evalin('base', 'fwd_kin_result');
        jacobian = Jacobian(fwd_kin_result); % Get Jacobian matrix
        jacobian_pinv = pinv(jacobian); % Get pseudo-inverse Jacobian matrix
        
        % Get rows 1-3 from 'delta' (dX; dY; dZ;) and divide by 10 to try
        % to move forward just 1/10th of the euclidean distance between the
        % start and goal positions
        d_xyz = delta(1:3) / 10;
        
        % Calculate a 'delta_theta' that allows the robotic arm to move
        % forward that 1/10th 
        delta_theta = jacobian_pinv * d_xyz;

        % Convert 'delta_theta' from radians to degrees
        delta_theta1 = radtodeg(delta_theta(1, 1));
        delta_theta2 = radtodeg(delta_theta(2, 1));
        delta_theta3 = radtodeg(delta_theta(3, 1));
        delta_theta4 = radtodeg(delta_theta(4, 1));

        % Add 'delta_theta' to each joint angle
        theta1 = theta1 + delta_theta1;
        theta2 = theta2 + delta_theta2;
        theta3 = theta3 + delta_theta3;
        theta4 = theta4 + delta_theta4;

        % Verify is we are still inside the configuration space for each
        % joint, after adding 'delta_theta'
        global motor1_max;
        global motor1_min;
        global servo2_max;
        global servo2_min;
        global servo3_max;
        global servo3_min;
        global servo4_max;
        global servo4_min;
        
        % If any of the new joint angles is outside its configuration space,
        % over its max angle limit, cancel the inverse kinematics
        % iterative calculation and go back to last valid joint angles
        
        % To flag outside of configuration space condition
        flag_outside_conf_space = false; 
        
        % Show errors in command window and set flag to true
        if(theta1 > motor1_max)
            disp(['Error: theta1 > motor1_max ', num2str(theta1), '>', num2str(motor1_max)]);
            flag_outside_conf_space = true; % We're outside of configuration space!

        elseif(theta2 > servo2_max)
            disp(['Error: theta2 > servo2_max ', num2str(theta2), '>', num2str(servo2_max)]);
            flag_outside_conf_space = true; % We're outside of configuration space!

        elseif (theta3 > servo3_max)
            disp(['Error: theta3 > servo3_max ', num2str(theta3), '>', num2str(servo3_max)]);
            flag_outside_conf_space = true; % We're outside of configuration space!
            
        elseif (theta4 > servo4_max)
            disp(['Error: theta4 > servo4_max ', num2str(theta4), '>', num2str(servo4_max)]);
            flag_outside_conf_space = true; % We're outside of configuration space!
        
        elseif (theta1 < motor1_min)
            disp(['Error: theta1 < motor1_min ', num2str(theta1), '<', num2str(motor1_min)]);
            flag_outside_conf_space = true; % We're outside of configuration space!

        elseif (theta2 < servo2_min)
            disp(['Error: theta2 < servo2_min ', num2str(theta2), '<', num2str(servo2_min)]);
            flag_outside_conf_space = true; % We're outside of configuration space!
            
        elseif (theta3 < servo3_min)
            disp(['Error: theta3 < servo3_min ', num2str(theta3), '<', num2str(servo3_min)]);
            flag_outside_conf_space = true; % We're outside of configuration space!
            
        elseif (theta4 < servo4_min)
            disp(['Error: theta4 < servo4_min ', num2str(theta4), '<', num2str(servo4_min)]);
            flag_outside_conf_space = true; % We're outside of configuration space!
        end

        % If we are outside of configuration space, we must go back to last
        % valid joint angles and cancel the inverse kinematics iterative
        % calculation
        if flag_outside_conf_space == true
            % Go back to last valid angles
            theta1 = theta1 - delta_theta1;
            theta2 = theta2 - delta_theta2;
            theta3 = theta3 - delta_theta3;
            theta4 = theta4 - delta_theta4;
            theta_start_invkin = [theta1; theta2; theta3; theta4];

            % Set inverse kinematics off and save intermid position as init
            % position for next inverse kinematics calculation. 
            % Save new start angles as well.
            assignin('base', 'invkin_calc_on', 0);
            intermid_pos = [fwd_kin_result(1, 16); 
                            fwd_kin_result(2, 16); 
                            fwd_kin_result(3, 16)];
            assignin('base', 'inv_kin_init_pos', intermid_pos);
            assignin('base', 'theta_start_invkin', theta_start_invkin);
            f = msgbox('Outside of configuration space', 'Error','error');
            break;
        end % End of: if flag_outside_conf_space == true

        % Calculate forward kinematics with last valid angles to display
        % the last valid pose in the GUI
        Run_Plot_Fwd_Kinematics(theta1, theta2, theta3, theta4);

        % Save new init position for next iteration (if inv. kin. is still on)
        intermid_pos = [fwd_kin_result(1, 16);
                        fwd_kin_result(2, 16); 
                        fwd_kin_result(3, 16)];

        % Compute new 'delta' between start and goal positions
        delta = Calc_Pos_Delta(intermid_pos, inv_kin_final_pos);

        % Compute the squared Euclidean error
        euclidean_error = delta(5, 1) ^ 2;

        % Update the GUI's error text label, showing a red back groud
        set(handles.txt_euclidean_error, ...
            'String', num2str(euclidean_error, 3), ...
            'BackgroundColor', 'Red');

        % Send new joint angles to the embedded controller, only if there's
        % at least a 'min_angle_change' (absolute value) in any of the new
        % joint angles

        % Min absolute change in joint angle. The new angles will be sent
        % to the embedded controller only if at least one of them have a
        % change of at least one degree (absolute value). This is because
        % the resolution of the servos is just 1 degree.
        min_angle_change = 1; 

        % Verify if there's at least a 'min_angle_change' in any joint
        % angle before sending the new angles to the embedded controller
        if( (abs(theta1-theta1_prev) >= min_angle_change) ...
            || (abs(theta2-theta2_prev) >= min_angle_change) ...
            || (abs(theta3-theta3_prev) >= min_angle_change) ...
            || (abs(theta4-theta4_prev) >= min_angle_change) )
        
            % Print to the command window the previous, current and delta
            % of the angles (just for debugging)
            fprintf('theta1_prev, theta1, theta1-theta1_prev: %.2f %.2f %.2f\n', [theta1_prev, theta1, theta1-theta1_prev]);
            fprintf('theta2_prev, theta2, theta2-theta2_prev: %.2f %.2f %.2f\n', [theta2_prev, theta2, theta2-theta2_prev]);
            fprintf('theta3_prev, theta3, theta3-theta3_prev: %.2f %.2f %.2f\n', [theta3_prev, theta3, theta3-theta3_prev]);
            fprintf('theta4_prev, theta4, theta4-theta4_prev: %.2f %.2f %.2f\n', [theta4_prev, theta4, theta4-theta4_prev]);
            
            % Print to the command window the data frames that will be sent
            % to the embedded controller
            disp('Inv. Kin: new angles to send');
            fprintf(strcat('1$', num2str(round(theta1)), '& '));
            fprintf(strcat('2$', num2str(round(theta2)), '& '));
            fprintf(strcat('3$', num2str(round(theta3)), '&'));
            fprintf(strcat('4$', num2str(round(theta4)), '&'));
            fprintf('\n');

            % Send joint angles through the serial port
            fprintf(serial_port, strcat('1$', num2str(round(theta1)), '&'));
            fprintf(serial_port, strcat('2$', num2str(round(theta2)), '&'));
            fprintf(serial_port, strcat('3$', num2str(round(theta3)), '&'));
            fprintf(serial_port, strcat('4$', num2str(round(theta4)), '&'));

            % Current angles are now previous angles for the next inverse
            % kinemaics calculation
            theta1_prev = theta1;
            theta2_prev = theta2;
            theta3_prev = theta3;
            theta4_prev = theta4;
        end % End of: if( (abs(theta1-theta1_prev) >= min_angle_change) || ...

        % Verify if the Euclidean error is less than a given maximum
        % tolerance to end the iterative inverse kinematics calculation
        if euclidean_error < 10^-3  
            disp('A maximum Euclidean error has been reached.')
            assignin('base', 'invkin_calc_on', 0);
            assignin('base', 'inv_kin_init_pos', intermid_pos);
            theta_start_invkin = [theta1; theta2; theta3; theta4];
            assignin('base', 'theta_start_invkin', theta_start_invkin);

            % Change the GUI's error text label background to green
            set(handles.txt_euclidean_error, 'BackgroundColor', 'Green');
        end

        % Verify if all errors (dX; dY; dZ; EucXY; EucXYZ; Zang) are less
        % than a given maximum tolerance to end the iterative inverse
        % kinematics calculation
        if all(delta(:) < 10^-3)
            disp('Se alcanzo error general minimo.')
            assignin('base', 'invkin_calc_on', 0);
            assignin('base', 'inv_kin_init_pos', intermid_pos);
            assignin('base', 'theta_start_invkin', theta_start_invkin);
            
            % Change the GUI's error text label background to green
            set(handles.txt_euclidean_error, 'BackgroundColor', 'Green');
        end 

    end % End of: while (evalin('base', 'invkin_calc_on') == 1)

    disp('Inverse kinematics iterative calculation has finished.');

    % Set new angle values to angle sliders and text labels in the GUI
    set(handles.txt_motor1_angle, 'String', num2str(theta1));
    set(handles.txt_servo2_angle, 'String', num2str(theta2));
    set(handles.txt_servo3_angle, 'String', num2str(theta3));
    set(handles.txt_servo4_angle, 'String', num2str(theta4));
    set(handles.slider_motor1_angle, 'Value', theta1);
    set(handles.slider_servo2_angle, 'Value', theta2);
    set(handles.slider_servo3_angle, 'Value', theta3);
    set(handles.slider_servo4_angle, 'Value', theta4);

    % Set new cordinate values to coordinate sliders and text labels in the GUI
    set(handles.txt_coordx, 'String', num2str(fwd_kin_result(1, 16)));
    set(handles.txt_coordy, 'String', num2str(fwd_kin_result(2, 16)));
    set(handles.txt_coordz, 'String', num2str(fwd_kin_result(3, 16)));
    set(handles.slider_coordx, 'Value', fwd_kin_result(1, 16));
    set(handles.slider_coordy, 'Value', fwd_kin_result(2, 16));
    set(handles.slider_coordz, 'Value', fwd_kin_result(3, 16));

    % Print to the command window the data frames that will be sent
    % to the embedded controller
    fprintf(strcat('1$', num2str(round(theta1)), '& '));
    fprintf(strcat('2$', num2str(round(theta2)), '& '));
    fprintf(strcat('3$', num2str(round(theta3)), '&'));
    fprintf(strcat('4$', num2str(round(theta4)), '&'));
    fprintf('\n');

    % Send joint angles through the serial port
    fprintf(serial_port, strcat('1$', num2str(theta1), '&'));
    fprintf(serial_port, strcat('2$', num2str(theta2), '&'));
    fprintf(serial_port, strcat('3$', num2str(theta3), '&'));
    fprintf(serial_port, strcat('4$', num2str(theta4), '&'));

end % End 'if inv_kin_final_pos == inv_kin_init_pos'


% --- Executes on button press in button_reset.
function button_reset_Callback(hObject, eventdata, handles)
% hObject    handle to button_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

axes(handles.cartesian_coordinates);
theta_init = evalin('base', 'theta_init'); % Get robotic arm's start pose
assignin('base', 'theta_start_invkin', theta_init); % Save start pose for inverse kinematics

% Plot start pose after computing forward kinematics
Run_Plot_Fwd_Kinematics(theta_init(1, 1), theta_init(2, 1), theta_init(3, 1), theta_init(4, 1));
% fwd_kin_result = evalin('base', 'fwd_kin_result');

% Initialization code for inverse kinematics
% Read last pose calculated using forward kinematics from 'base' workspace
% See the 'Run_Plot_Fwd_Kinematics' file to see
fwd_kin_result = evalin('base', 'fwd_kin_result');

% Set initial and final position for inverse kinematics as the same
inv_kin_init_pos = [fwd_kin_result(1, 16); fwd_kin_result(2, 16); fwd_kin_result(3, 16)];
inv_kin_final_pos = inv_kin_init_pos;

% Save both to 'base' workspace
assignin('base', 'inv_kin_init_pos', inv_kin_init_pos);
assignin('base', 'inv_kin_final_pos', inv_kin_final_pos);

% Assign the new coordinate values obtained from forward kinematics
set(handles.txt_coordx, 'String', num2str(fwd_kin_result(1, 16)));
set(handles.txt_coordy, 'String', num2str(fwd_kin_result(2, 16)));
set(handles.txt_coordz, 'String', num2str(fwd_kin_result(3, 16)));
set(handles.slider_coordx, 'value', fwd_kin_result(1, 16));
set(handles.slider_coordy, 'value', fwd_kin_result(2, 16));
set(handles.slider_coordz, 'value', fwd_kin_result(3, 16));

% Re-init forward kinematics with default angle values
assignin('base', 'theta_start_invkin', theta_init); 
set(handles.txt_motor1_angle, 'String', num2str(theta_init(1)));
set(handles.txt_servo2_angle, 'String', num2str(theta_init(2)));
set(handles.txt_servo3_angle, 'String', num2str(theta_init(3)));
set(handles.txt_servo4_angle, 'String', num2str(theta_init(4)));
set(handles.slider_motor1_angle, 'Value', theta_init(1)); 
set(handles.slider_servo2_angle, 'Value', theta_init(2)); 
set(handles.slider_servo3_angle, 'Value', theta_init(3));
set(handles.slider_servo4_angle, 'Value', theta_init(4)); 

% Send start angles to the robotic arm
global serial_port; % Global variable referencing the serial port
fprintf(serial_port, strcat('1$', num2str(theta_init(1)), '&'));
fprintf(serial_port, strcat('2$', num2str(theta_init(2)), '&'));
fprintf(serial_port, strcat('3$', num2str(theta_init(3)), '&'));
fprintf(serial_port, strcat('4$', num2str(theta_init(4)), '&'));


% --- Executes on button press in checkbox_gripper_state.
function checkbox_gripper_state_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_gripper_state (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_gripper_state
gripper_value = get(hObject,'Value');
disp(gripper_value);

global serial_port; % Global variable referencing the serial port
global servo_grip_close;
global servo_grip_open;

% Check if the gripper was oopen or closed in the GUI
if(gripper_value)
    disp('Closing the gripper...');
    fprintf(strcat('5$', num2str(servo_grip_close), '&'));
    fprintf(serial_port, strcat('5$', num2str(servo_grip_close), '&'));
    
    % Modify Gripper state in GUI
    set(handles.checkbox_gripper_state, ...
        'String', 'Gripper is Closed', ...
        'ForegroundColor', 'Red');
else
    disp('Opening the gripper...');
    fprintf(strcat('5$', num2str(servo_grip_open), '&'));
    fprintf(serial_port, strcat('5$', num2str(servo_grip_open), '&'));
    
    % Modify Gripper state in GUI
    set(handles.checkbox_gripper_state, ...
        'String', 'Gripper is Open', ...
        'ForegroundColor', [0 0.5 0]); % Dark green
end
% --- Executes on slider movement.
function slider_servo4_angle_Callback(hObject, eventdata, handles)
% hObject    handle to slider_servo4_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get modified angle for current motor and display it in the corresponding
% GUI's text label
angle4 = get(hObject, 'Value');
set(handles.txt_servo4_angle, 'String', num2str(angle4, 3));

% Get angles from the rest of the motors
angle1 = str2num(get(handles.txt_motor1_angle, 'String'));
angle2 = str2num(get(handles.txt_servo2_angle, 'String'));
angle3 = str2num(get(handles.txt_servo3_angle, 'String'));

% Create angles array
angles = [angle1; angle2; angle3; angle4];

% Compute forward kinematics and plot resulting pose in the GUI
Run_Plot_Fwd_Kinematics(angle1, angle2, angle3, angle4);
view(evalin('base', 'coord_3d_perspective'));

% -- Send new joint angles via serial communications
% Print angle data frames to command window
disp('Servo 3 angle changed, sending new angles:');
fprintf(strcat('1$', num2str(round(angle1)), '&'));
fprintf(strcat('2$', num2str(round(angle2)), '&'));
fprintf(strcat('3$', num2str(round(angle3)), '&'));
fprintf(strcat('4$', num2str(round(angle4)), '&'));

% Send angles through the serial port
global serial_port; 
fprintf(serial_port,strcat('1$', num2str(round(angle1)), '&'));
fprintf(serial_port,strcat('2$', num2str(round(angle2)), '&'));
fprintf(serial_port,strcat('3$', num2str(round(angle3)), '&'));
fprintf(serial_port,strcat('4$', num2str(round(angle4)), '&'));

% Read the forward kinematics calculation result to update GUI's coordinate
% sliders and text labels
fwd_kin_result = evalin('base', 'fwd_kin_result');

% Set new values in corresponding sliders and text labels
set(handles.txt_coordx, 'String', num2str(fwd_kin_result(1, 16)));
set(handles.txt_coordy, 'String', num2str(fwd_kin_result(2, 16)));
set(handles.txt_coordz, 'String', num2str(fwd_kin_result(3, 16)));
set(handles.slider_coordx, 'value', fwd_kin_result(1, 16));
set(handles.slider_coordy, 'value', fwd_kin_result(2, 16));
set(handles.slider_coordz, 'value', fwd_kin_result(3, 16));

% Update new init position for next inverse kinematics calculation
inv_kin_init_pos = [fwd_kin_result(1, 16); 
                    fwd_kin_result(2, 16); 
                    fwd_kin_result(3, 16)];
assignin('base', 'inv_kin_init_pos', inv_kin_init_pos);
assignin('base', 'theta_start_invkin', angles); % Start angles for inverse kinematics

% --- Executes during object creation, after setting all properties.
function slider_servo4_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_servo4_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
