coppelia=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
coppelia.simxFinish(-1); % just in case, close all opened connections
clientID=coppelia.simxStart('127.0.0.1',19999,true,true,5000,5);

row_size =4;
column_size = 4;
gridSize = row_size*column_size;
num_orientations = 4;
global OptimalPolicy
OptimalPolicy = zeros(gridSize,num_orientations,2);

if(clientID ~= -1)
    disp('Connected to Coppelia');
    
    % Now try to retrieve data in a blocking fashion (i.e. a service call):
    [res,objs]=coppelia.simxGetObjects(clientID,coppelia.sim_handle_all,coppelia.simx_opmode_blocking);
    
    %% Orientation
    % 1 - 0degree, 2 - 45degree ......... , 8 - 315 degree
    %% Index - 1 to 121 --- grid indices, more like index 1 to 16 as 4*4 is 16 
    goalIndex1 = 15;
    goalOrientation1 = 2;
    goalAngle1 = (goalOrientation1-1) * 90;
    
    botHeight = 0.1386;
    
    r_size = 4;
    gridResolution = 0.5;
    x_goal1 = (round(mod((goalIndex1-1),r_size))) * gridResolution;
    y_goal1 = (floor((goalIndex1-1)/r_size)) * gridResolution;
    goalPosition1 = [x_goal1, y_goal1, botHeight];
    
    goalIndex2 = 16;
    goalOrientation2 = 5;
    goalAngle2 = (goalOrientation2-1) * 90;
    x_goal2 = (round(mod((goalIndex2-1),r_size))) * gridResolution;
    y_goal2 = (floor((goalIndex2-1)/r_size)) * gridResolution;
    goalPosition2 = [x_goal2, y_goal2, botHeight];
    
    %% Get the optimal policy through MDP
    MDP(goalIndex1,goalOrientation1, 1);
%     MDP(goalIndex2,goalOrientation2, 2);
    
    % start the simulation:
    coppelia.simxStartSimulation(clientID,coppelia.simx_opmode_blocking);
    
    %% Get reference frame handle
    [returnCode,RefFrame]=coppelia.simxGetObjectHandle(clientID,'/ReferenceFrame',coppelia.simx_opmode_blocking);
    [returnCode,RefFrameBody1]=coppelia.simxGetObjectHandle(clientID,'/Pioneer3DX/ReferenceFrame',coppelia.simx_opmode_blocking);
%     [returnCode,RefFrameBody2]=vrep.simxGetObjectHandle(clientID,'ReferenceFrame1',coppelia.simx_opmode_blocking);
    
    %% Get the bot handle
    [returnCode,Pioneer_P3DX]=coppelia.simxGetObjectHandle(clientID,'/PioneerP3DX',coppelia.simx_opmode_blocking);
%     [returnCode,BotHandle2]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',coppelia.simx_opmode_blocking);
    
    %% Get motor handles  
    [returnCode,Pioneer_P3DX_LeftMotor]=coppelia.simxGetObjectHandle(clientID,'/PioneerP3DX/leftMotor',coppelia.simx_opmode_blocking);
    [~,Pioneer_P3DX_RightMotor]=coppelia.simxGetObjectHandle(clientID,'/PioneerP3DX/rightMotor',coppelia.simx_opmode_blocking);
%     
%     [returnCode,LeftMotorHandle2]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',coppelia.simx_opmode_blocking);
%     [~,RightMotorHandle2]=coppelia.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',coppelia.simx_opmode_blocking);
%     
    %% Get the current position and orientation of the robot
    [returnCode,position1]=coppelia.simxGetObjectPosition(clientID, Pioneer_P3DX, RefFrame, coppelia.simx_opmode_blocking);
    [returnCode,eulerAngles1]=coppelia.simxGetObjectOrientation(clientID,RefFrameBody1,RefFrame,coppelia.simx_opmode_blocking);
        
    currentPosition1 = position1;
    currentEulerAngles1 = eulerAngles1;
     
%     [returnCode,position2]=coppelia.simxGetObjectPosition(clientID, BotHandle2, RefFrame, coppelia.simx_opmode_blocking);
%     [returnCode,eulerAngles2]=coppelia.simxGetObjectOrientation(clientID,RefFrameBody2,RefFrame,coppelia.simx_opmode_blocking);
% 
%     currentPosition2 = position2;
%     currentEulerAngles2 = eulerAngles2;
    
    ReachedGoal1 = false;
    ReachedGoal2 = false;
    %% Loop till we reach at goal
    while ReachedGoal1 ~= true %|| ReachedGoal2 ~= true
        
        if(ReachedGoal1 ~= true)
            [ReachedGoal1, currentPosition1, currentEulerAngles1] = MoveRobot(clientID, coppelia, Pioneer_P3DX, Pioneer_P3DX_LeftMotor, Pioneer_P3DX_RightMotor, currentPosition1, currentEulerAngles1, goalPosition1, goalAngle1, 1, RefFrame, RefFrameBody1);
        end
        
%         if(ReachedGoal2 ~= true)
%             [ReachedGoal2, currentPosition2, currentEulerAngles2] = MoveRobot(clientID, coppelia, BotHandle2, LeftMotorHandle2, RightMotorHandle2, currentPosition2, currentEulerAngles2, goalPosition2, goalAngle2, 2, RefFrame, RefFrameBody2);
%         end

    end
    
    %% When complete stop the simulation
    coppelia.simxStopSimulation(clientID,coppelia.simx_opmode_blocking);
    
    %% Noisy GPS not used
    %     [res,data]=vrep.simxGetStringSignal(clientID,'myPts',vrep.simx_opmode_streaming);
    %     while(1)
    %         %% Get the GPS Data
    %         vrep.simxGetStringSignal(clientID,'myGpsData',vrep.simx_opmode_streaming); % Initialize streaming
    %         while (1)
    %             [returnCode,data]=vrep.simxGetStringSignal(clientID,'myGpsData',vrep.simx_opmode_buffer); % Try to retrieve the streamed data
    %             if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    %                 gpsData=vrep.simxUnpackFloats(data);
    %                 %% Get the position of robot
    %                 [returnCode,BotHandle]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#0',vrep.simx_opmode_blocking);
    %                 [returnCode,position]=vrep.simxGetObjectPosition(clientID, BotHandle, RefFrame, vrep.simx_opmode_blocking);
    %
    %             end
    %         end
    %     end
    %
    
    coppelia.simxFinish(clientID);
end


function [ReachedGoal, currentPosition, currentEulerAngles] = MoveRobot(clientID, coppelia, Pioneer_P3DX, Pioneer_P3DX_LeftMotor, Pioneer_P3DX_RightMotor, currentPosition, currentEulerAngles, goalPosition, goalAngle, roboNumber, RefFrame, RefFrameBody)

       if abs(currentPosition(1)-goalPosition(1)) < 0.3 && abs(currentPosition(2)-goalPosition(2)) < 0.3 && abs(goalAngle-((currentEulerAngles(3)*180)/pi)) < 25
           ReachedGoal = false;
           return;
       end
           
        % calculate the orientation
        orientation = (currentEulerAngles(3) * 180) / pi;
        RotationTimeConstant = 30;   % degrees / sec
        
        RightTurn = false;
        LeftTurn = false;
        
        StraighMotionFactor = 1;
        
        % Get the angle by which we need to turn
        deltaOrientation = GetAction(currentPosition(1), currentPosition(2), orientation, roboNumber);
        
        newOrientation = deltaOrientation + orientation;
        newOrientation = round(newOrientation/45)*45;
        Isdiagonal = false;
        if newOrientation == 90 || newOrientation == 180 || newOrientation == 270 %|| newOrientation == 315
            StraighMotionFactor = 1.414;
        end
        
        
        if(deltaOrientation < 180)
            LeftTurn = true;
        else
            RightTurn = true;
            deltaOrientation = 360-deltaOrientation;
        end
        
        time = 1.8 * deltaOrientation / RotationTimeConstant ;
        Speed = 0.5;
        
        if (RightTurn == true)
            %% Zero-radius right turn
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_LeftMotor,Speed,coppelia.simx_opmode_blocking);
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_RightMotor,-Speed,coppelia.simx_opmode_blocking);
            
            pause(time);
            
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_LeftMotor,0,coppelia.simx_opmode_blocking);
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_RightMotor,0,coppelia.simx_opmode_blocking);
            
            [returnCode,eulerAngles]=coppelia.simxGetObjectOrientation(clientID,Pioneer_P3DX,RefFrame,coppelia.simx_opmode_blocking);
        end
        if (LeftTurn == true)
            %% Zero-radius left turn
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_LeftMotor,-Speed,coppelia.simx_opmode_blocking);
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_RightMotor,Speed,coppelia.simx_opmode_blocking);
            
            pause(time);
            
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_LeftMotor,0,coppelia.simx_opmode_blocking);
            [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_RightMotor,0,coppelia.simx_opmode_blocking);
            
            [returnCode,eulerAngles]=coppelia.simxGetObjectOrientation(clientID,Pioneer_P3DX,RefFrame,coppelia.simx_opmode_blocking);
            
        end
        
        %% Straight motion
        [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_LeftMotor,Speed,coppelia.simx_opmode_blocking);
        [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_RightMotor,Speed,coppelia.simx_opmode_blocking);
        
        pause(11 * StraighMotionFactor);
        
        [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_LeftMotor,0,coppelia.simx_opmode_blocking);
        [returnCode]=coppelia.simxSetJointTargetVelocity(clientID,Pioneer_P3DX_RightMotor,0,coppelia.simx_opmode_blocking);
        
        %% Get the  current position and orientation of the robot
        [returnCode,currentPosition]=coppelia.simxGetObjectPosition(clientID, Pioneer_P3DX, RefFrame, coppelia.simx_opmode_blocking);

        [returnCode,currentEulerAngles]=coppelia.simxGetObjectOrientation(clientID,RefFrameBody, RefFrame,coppelia.simx_opmode_blocking);
        if (currentEulerAngles(3) < 0)
            currentEulerAngles(3) = currentEulerAngles(3) + (2*pi);
        end

       if abs(currentPosition(1)-goalPosition(1)) > 0.3 || abs(currentPosition(2)-goalPosition(2)) > 0.3 || abs(goalAngle-((currentEulerAngles(3)*180)/pi)) > 25
           ReachedGoal = false;
       else
           ReachedGoal = true;
       end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Collision Detect Function : 

function obs_flag = CollisionDetect(obstacles,x_new,y_new,row_size,gridResolution)
                       
% UNTITLED Summary of this function goes here
% Assumes there is no collision initially 
obs_flag = 0;

% this starts a loop that iterates over each obstacle 
for i=1:length(obstacles)

    %this line retrieves the index of the current obstacle being checked
    idx = obstacles(i);

    %calculates the y coordinate of the current obstacle based on its index
    %and grid properties , floor - ensures that the calculated y
    %corresponds to bottom edge of the cell grid 
    y_obs = (floor((idx-1)/row_size)) * gridResolution;

    % calculates the x coordinate of current obstacle 
    % round is used to get to the nearest integer value 
    x_obs = (round(mod((idx-1),row_size))) * gridResolution;
    
    % this checks if the new point coincides with coordinates of the
    % current obstacle 
    if (x_new == x_obs) && (y_new == y_obs) 
        % it returns this if there is a collisionn 
        obs_flag = 1;
    end
end
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Get Action function : 

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MDP

  
function[] = MDP(goalIndex,goalOrientation, roboNumber)

global OptimalPolicy

row_size = 4; % numbers of rows in the grid 
column_size = 4; % number of columns in the grid 
gridSize = row_size*column_size; % total size of the grid 
gridResolution = 0.5; 

% defines the number of states , actions and possible orientations 
num_actions = 4;
num_states = 11;
num_orientations = 4;

% initialsies the value fucntion matrix 'V' with dimensions gridsize by   
% num_orientations and fills it with -1 
V  = -1 .* zeros(gridSize,num_orientations);

% reads in transition probabilities from excel file 
%TP_temp = xlsread('TP.xlsx','Sheet1','A1:H8');
% V  = zeros(gridSize,num_orientations);

vel = 0.5; % defines the velocity of the robot 

% initialises the action spae matrix with different velocities in x and y
% directions, different combinations of speeds in x and y direction so for
% example : x- 0 , y - 0,5, or x-0.5 and y - 0.5.  - these define the
% different actions the robot can take in terms of directions and speed

actionSpace = [0,vel;
    -vel,vel;
    -vel,0;
    -vel,-vel;
    0,-vel;
    vel,-vel;
    vel,0;
    vel,vel];

%option to generate random obstacles in the grid 
%Obstacles = randi([0 gridSize],1,ceil(0.5*row_size));
%position of obstacles in the grid 
Obstacles = [3,5,11,13,14];

%%%
% Define obstacle positions (example)
obstacle_positions = [3, 1; 1, 2; 1, 4; 3, 3; 2, 4];  %Row and column indices of obstacle cells

%define the probabiltiies for correct and incorrect actions 
cor_pr = 0.8; %prob of correct action 
wr_pr = 0.2; %prob of wrong action 

%%%
%Call the function to generate the transition probability matrix 
%TransitionProbabilities = build_st_trans_matrix();
TransitionProbabilities = build_st_trans_matrix(row_size, column_size, num_actions, cor_pr, wr_pr, obstacle_positions);
%TransitionProbabilities = build_st_trans_matrix(4, 4, 8, 0.8, 0.2, [3, 1; 1, 2; 1, 4; 3, 3; 2, 4]);
 
%to find and remove the goal position from the list of obstacles 
%goal_idx = find(Obstacles(:) == goalIndex);
%Obstacles(goal_idx) = [];

goal_idx = find(Obstacles(:) == goalIndex);
Obstacles(goal_idx) = [];

% V = CreateObstacle(V,Obstacles);
% this sets the value of the goal state in the value function matrix V to 0
V(goalIndex,goalOrientation) = 0;

V_pre = V ; % initiliases a previous value as 'V'

%additional line question-mark 
OptimalPolicy = zeros(gridSize, num_orientations, roboNumber);
%This line initialises the optimal policy for the goal state 
OptimalPolicy(goalIndex,goalOrientation, roboNumber) = 0;

% based on the given transition probabilities and rewards the value
% function V and optimal policy is updated to find otpimal policy to reach
% foal state whilst avoiding obstacles

%for i=1:num_actions
    % this initiliases the transitionprob matrix that represents the
    % transition from one state to another for each action 
    % It iterates from 1 ti total number of actions 
    %TransitionProbabilities(i,:) = zeros(1,num_states); % for current action i this initialisesthe transition probabilities, a d sets all transition probabilities from current action for each state to 0 
    % the zeros , creates a row vector of zeros with length equal to no. of
    % states, creates a row vector of zeros 

%end
  
%TransitionProbabilities = TP_temp;% assigns tthe transitn prob matrix from excel to variable 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





% Define a function for transition probabilities 

function TransitionProbabilities = build_st_trans_matrix(row_size, column_size, num_actions, cor_pr, wr_pr, obstacle_positions)
        % Calculate the total number of states
        num_states = row_size * column_size;

        % Initialize transition probability matrix
        TransitionProbabilities = zeros(num_actions, num_states, num_states);

        % Loop through each cell in the grid
        for row = 1:row_size
            for col = 1:column_size
                cur_state = (row - 1) * column_size + col;

                % Check if the current cell is an obstacle
                if any(ismember(obstacle_positions, [row, col], 'rows'))
                    % Skip processing for obstacle cells
                    continue;
                end

                % Define neighboring cells
                right_state = (row - 1) * column_size + min(col + 1, column_size);
                top_state = max(row - 1, 1) * column_size + col;
                left_state = (row - 1) * column_size + max(col - 1, 1);
                bottom_state = min(row + 1, row_size) * column_size + col;

                % Define action map for each cell
                action_map = [right_state, top_state, left_state, bottom_state];

                % Loop through each action
                for action = 1:num_actions
                    % Rotate action map based on the current action
                    action_map_rot = circshift(action_map, [0, action - 1]);

                    % Assign transition probabilities
                    for inner_action = 1:num_actions
                        if inner_action == 1
                            TransitionProbabilities(action, cur_state, action_map_rot(inner_action)) = cor_pr;
                        else
                            TransitionProbabilities(action, cur_state, action_map_rot(inner_action)) = wr_pr;
                        end
                    end
                end
            end
        end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Define Transformation Function 

function TransformationMatrix = Transformation(orientation)
    orientation = mod(orientation-1,4)+1;
    disp("orientation");
    disp(orientation);
   %Define the rotation angles for each orientation (assuming 4 directions)
    rotationAngles = [0, 90, 180, 270]; %Degrees 

    % Get roation angle based on orientation 
    disp("rotation angle");
    disp(rotationAngles(orientation));

    rotationAngle = rotationAngles(orientation);

    %Create a 2D rotation matrix for the given angle
    TransformationMatrix= [cosd(rotationAngle), -sind(rotationAngle); sind(rotationAngle), cosd(rotationAngle)];
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Angle = 0:45:315; % an array of discrete angle from 0 to 315 deg w/ step size of 45 deg
Angle = 0:90:270; % Degrees 

eps = 1E5; % tracks the convergence of value it process 
Conv = 1E-4; % represents convergence criterion

% VALUE Iteration loop - iterates until convergence criterion is met 
% to iterate convergence until criterion is met 
while eps > Conv
    % nested loops iterate over each grid cell idx and orientation
    for idx = 1:gridSize
        for orient =1:num_orientations
            % current position is calculated
            y_curr = (floor((idx-1)/row_size)) * gridResolution;
            x_curr = (round(mod((idx-1),row_size))) * gridResolution;
            
            tempV = zeros(num_actions,1);% temp value to store expected return for each action 
            
            % nested loops iterate over each possible action 
            for action = 1:num_actions
                % calculates the local action for the current orientation
                %sizeTransformation = size(Transformation(orient));
                %sizeActionSpace = size(actionSpace(action, :));

                %disp('Size of Transformation(orient):');
                %disp(sizeTransformation);

                %disp('Size of actionSpace(action, :)');
                %disp(sizeActionSpace);

                localAction = Transformation(orient)* actionSpace(action,:)';
                localAction = (localAction(:,:));% ensures 'localAction is treated as a column vector 
                
                % iterated over reach possible state 
                for state = 1:num_states
                    S_Transformation = Transformation(state);
                    disp(S_Transformation);
                    check_extent =0;
                    next_state = zeros(2,1);
              
                    %sizeTransformation = size(S_Transformation);
                    %sizelocalAction = size(localAction );

                    %disp(sizeTransformation);
                    %disp(sizelocalAction);

                    %new_vector = Transformation(state) * localAction;
                    new_vector = S_Transformation* localAction;

                    new_vector = (new_vector(:,:));
                    % checks if the new vecotr is eqaul to the local action
                    if new_vector == localAction % if condiiton is met the actions is in the cardinal direction and next state cna be computed directly
                        x_new = x_curr + localAction(1);
                        y_new = y_curr + localAction(2);
                        x_new = round(x_new/vel)*vel;
                        y_new = round(y_new/vel)*vel;
                    else % if not in cardinal direction like above but in the diagonal direction the next state is computed , new coordinates are computed  
                        x_new = x_curr + new_vector(1); % new coordinates are computed based on current position 
                        y_new = y_curr + new_vector(2);
                        x_new = round(x_new/vel)*vel; % coordinates are rounded up to align with grid and robots velocity 
                        y_new = round(y_new/vel)*vel;
                    end
                    % if the new state is within the grid boundaries, the
                    % code checks for obstacles and computes rewards 
                    if (x_new <0 || x_new >= (row_size*gridResolution)) || (y_new <0 || y_new >= (column_size*gridResolution))
                        check_extent = 1; % if it doesnt le in grid boundaries it sets the check extent to 1 
                    end
                    
                    if check_extent ~= 1 % if the next state is within the grid boundaries then it calculates the properties of the state 
                        
                        IsObstacle = 0; % initialises vairbale to check if an obstacle is encountered in next state 
                        % calculates the row index and the column index of
                        % the next state in x and y direction 
                        next_state(1) = floor(y_new * row_size / gridResolution);
                        next_state(1) = next_state(1)+(x_new/gridResolution)+1;
                        
                        %calculates the index of the next states
                        %orientation based on angles of current action,
                        %state, orientation. - changed value from 8 to 4 as
                        % 4 orientations 
                        %disp(Angle(action));
                        %disp(Angle(state));
                        %disp(Angle(orient));

                        %temp = int16((Angle(action)+Angle(state)+Angle(orient))/90)+1;
                        temp = int16((Transformation(action)+Transformation(state)+Transformation(orient))/90)+1; 


                        while temp >4
                            temp = temp-4;
                            disp(temp);
                            next_state(2) = temp;
                        end
                        
                        %this line assigns the calculates orientation index
                        %to next state 
                        sizetemp = size(temp);
                        %disp('size of temp');
                        %disp(class(temp));
                        %disp(sizetemp);
                        nextstate_2 = next_state(2);
                        %disp('size of next state');
                        %disp(size(nextstate_2));
                        %disp(class(nextstate_2));
                        %disp(sizenexstate);
                        %the size of temp is 2 2 and the size of next state
                        %is 1 1 therefore need to change the siz 
                        %temp - int16 , nextstate - double 
                        %next_state(2) = temp;
                        next_state(2) = temp(1,1);

                        % checks if next state encounters obstacle by
                        % calling the collision detect function 
                        IsObstacle = CollisionDetect(Obstacles,x_new,y_new,row_size,gridResolution);
                        
                        %reward is set according to whether a goal ,
                        %obstacle is encountered 
                        if next_state(1) == goalIndex && next_state(2) == goalOrientation
                            Reward = 1;
                        elseif IsObstacle == 1
                            Reward = -10;
                        else
                            Reward = 0;
                        end

                        % Updates the expected return for current action ,
                        % In temp value fn vector 'tempV'
                        % it calculates the expected return based on
                        % transition probabilities,reward and value for next
                        % state 

                        %display the contents of the next array
                        %disp('Contents of next_state array:');
                        %disp(next_state);
                        %disp(next_state(1));
                        %disp(next_state(2));

                        %display contents of V_pre 
                        %disp('Contents of V_Pre');
                        %disp(V_pre);
                        %disp(V_pre(next_state(1)));
                        %disp(V_pre(next_state(2)));
                        %disp(V_pre(next_state(1),next_state(2)));
                        %display contents of V_pre

                        tempV(action) = tempV(action) + TransitionProbabilities(action,state)*(Reward + V_pre(next_state(1),next_state(2)));
 
                    end
                end
            end
            % to update the value function and optimal policy based on
            % highest expected return obtained during value iteration
            % process. Ensures robot learns best actions to take in each
            % state the maximise its rewards
            [compVal,maxIndex] = max(tempV);% find max value comp value  and its index maxindex 
            if compVal > V_pre(idx,orient) % if the highest expected erturn for the current stae adn orientation is greater than the previous value stored 
                V(idx,orient) = compVal; 
                OptimalPolicy(idx,orient, roboNumber) = maxIndex;% so if it has improved the vlaue functiona nd optimal policy will have to be updated - policy improvement 
            end
            V(goalIndex,goalOrientation) = 0; %sets the value function of the goal state to zero
            OptimalPolicy(goalIndex,goalOrientation, roboNumber) = 0; %This line sets the optimal policy for the goal state to zero. no action is eneded as episode has ended 
        end
    end
    % check the convergence if value iteration algorithm, abs difference
    % between current and previous value functions 'eps'
    eps = abs(max(max(V -V_pre)));
    V_pre = V;
end

% Now integrate synchronous mode here:
    disp('Program started');
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID > -1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        sim.simxSynchronous(clientID,true);

        % start the simulation:
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);

        % Now step a few times:
        for i=0:10
            disp('Press a key to step the simulation!');
            pause;
            sim.simxSynchronousTrigger(clientID);
        end

        % stop the simulation:
        sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end
% value iteration algorithm to iteratively update the value function and optimal policy for robot navigation in a grid environment, 
% ensuring that the robot learns the best actions to maximize its cumulative rewards over time.
