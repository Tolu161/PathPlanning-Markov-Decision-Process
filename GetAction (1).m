% function that takes the inputs x_cur and y_curr and more and returns one
% output variable next_angle 

%it computes the next angle for a robot based on the current position
%,angle and predefined policy
function [next_angle] = GetAction(x_curr,y_curr,curr_angle, roboNumber)
global OptimalPolicy;

%defines the size of the grid and the resolution of the grid 
r_size = 11;
gridResolution = 0.5;

% rounds the current x-coordinate to the nearest multiple of the
% grid resolution ensures robot position aligns with the grid
x_curr = round(x_curr/gridResolution)*gridResolution;
y_curr = round(y_curr/gridResolution)*gridResolution;

%calculates index of current position in grid on y coordinate 
curr_idx = floor(y_curr * r_size / gridResolution);
%adjusts the current index adding the coord and1 . to start from 1 not 0 
curr_idx = curr_idx+(x_curr/gridResolution)+1;

% rounds current angle to nearest 45 deg , for further processing later 
curr_angle = round(curr_angle/45)*45;

% ensures that angle range is between 0 and 315 
if(curr_angle > 315)
    curr_angle = 0;
end

% an array of all possible angles between 0 & 315 with stepsize of 45 deg 
Angle = 0:45:315;

% it finds the index of the current angle in the ANGLE array 
curr_angle_idx = find(Angle(:)==curr_angle);

%based on the current angle and the index of that angle in the array, it
%retrieves the next angle from policy lookup table in optimal policy based
%on the current position index , robonumnber and angle 
next_angle_idx= OptimalPolicy(curr_idx,curr_angle_idx, roboNumber);

% calculation of the next angle to convetrt angle index to the associated degrees 
next_angle = (next_angle_idx-1)*45;


end