   
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
