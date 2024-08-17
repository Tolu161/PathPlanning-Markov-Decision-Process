% MDPgridworldExample function definition
function policyitexample()
    % Define the grid world
    set(0,'DefaultAxesFontSize',18)
    pauseOn = false;  %setting this to 'true' is useful for teaching, because it pauses between each graph
    World = [  %this world is asymmetrical, so it can be used to talk about POMDPS
        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
        1 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 
        1 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1
        1 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1
        1 0 0 0 0 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 0 1
        1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 0 0 0 1
        1 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 1
        1 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1
        1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1
        1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1
        1 0 0 0 1 1 1 0 0 0 0 0 0 1 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1
        1 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
        1 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
        1 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];
    
    f1 = figure(1); clf
    set(f1,'units','normalized','outerposition',[0 0 1 1])
    % Define the reward function
    R = -50 * ones(size(World)); %-50 reward for obstacles
    R(World == 0) = -1; %small penalty for not being at the goal
    %R(8, 11) = 100; %goal state has big reward
    R(6, 49) = 100; %goal state has big reward
 
   
%  DRAW THE WORLD, REWARD, ANIMATE POLICY ITERATION, DISPLAY POLICY
subplot(2,2,1)
imagesc(~World);
set(gca,'Xtick',[], 'Ytick',[])
axis equal
axis tight
text(25,-1,'World','HorizontalAlignment','center','FontSize',18)
drawnow
if pauseOn; pause(); end %#ok<*UNRCH>

subplot(2,2,2)
imagesc(R);
axis equal
axis tight
set(gca, 'Xtick',[], 'Ytick',[])
text(25,-1,'Reward function','HorizontalAlignment','center','FontSize',18)
drawnow
if pauseOn; pause(); end

% Apply policy iteration to learn a policy
%policy = policy_iteration_multiple_iterations(R, World, 0.97,200); % Call policy iteration function
[policy, iterationTimes,totalReward, elapsedTime] = policy_iteration_multiple_iterations(R, World, 0.97, 200);

%policy = policy_iteration_stochastic(R, World, 0.97,true); % Call policy iteration function

% Display the learned policy
DrawPolicy(policy, World, false);

policy = policy_iteration_multiple_iterations(R,World,0.97,false);
if pauseOn; pause(); end
DrawPolicy(policy,World,false);
if pauseOn; pause(); end
figure(f1)
policy_prob = policy_iteration_multiple_iterations(R,World,0.97,true);
if pauseOn; pause(); end
DrawPolicy(policy_prob,World,true);
if pauseOn; pause(); end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% new figures added 

% Ensure figures are created in a visible state
figure('Name', 'Average Time Per Value Update Iteration', 'NumberTitle', 'off', 'Visible', 'on');
subplot(3,1,1);
plot(iterationTimes);
title('Average Time Per Value Update Iteration');
xlabel('Iteration');
ylabel('Time (seconds)');

figure('Name', 'Total Computation Time Over Iterations', 'NumberTitle', 'off', 'Visible', 'on');
subplot(3,1,2);
plot(cumsum(iterationTimes));
title('Total Computation Time Over Iterations');
xlabel('Iteration');
ylabel('Cumulative Time (seconds)');

figure('Name', 'Total Reward Collected Over Time', 'NumberTitle', 'off', 'Visible', 'on');
subplot(3,1,3);
plot(totalReward);
title('Total Reward Collected Over Time');
xlabel('Iteration');
ylabel('Total Reward');

%%%%%Additional Policy iteration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [policy, iterationTimes, totalReward, elapsedTime] = policy_iteration_multiple_iterations(R, World, gamma, maxIterations)
    % Initialize policy randomly
    policy = randi([1, 9], size(World));
    
    % Convert obstacles to walls in policy
    policy(World == 1) = 0;
    
    % Define possible moves
    moves = [1, 0; 1, 1; 0, 1; -1, 1; -1, 0; -1, -1; 0, -1; 1, -1; 0, 0]; 
    
    iterationCount = 0;
    iterationTimes = [];  % Array to store the time of each iteration

    totalReward = zeros(1, maxIterations);  % Array to store total reward per iteration
    
    % Measure total elapsed time
    totalStartTime = tic;
    
    % Iterate until policy converges or maxIterations reached
    while iterationCount < maxIterations
        % Start timing this iteration
        startTime = tic;
        

        % Policy evaluation
        V = zeros(size(World));
        V(World == 1) = -Inf;
        
        delta = Inf;
        while delta > 0.01
            delta = 0;
            for i = 1:size(World, 1)
                for j = 1:size(World, 2)
                    if World(i, j) == 0
                        % Compute value for each action
                        action_values = zeros(1, 9);
                        for action = 1:9
                            next_state = [i, j] + moves(action, :);
                            if all(next_state >= 1) & all(next_state <= size(World)) & World(next_state(1), next_state(2)) == 0
                                action_values(action) = R(next_state(1), next_state(2)) + gamma * V(next_state(1), next_state(2));
                            else
                                action_values(action) = -Inf;
                            end
                        end
                        % Update value of current state
                        new_V = max(action_values);
                        delta = max(delta, abs(new_V - V(i, j)));
                        V(i, j) = new_V;
                    end
                end
            end
        end
        
        % Policy improvement
        policy_stable = true;
        totalReward(iterationCount + 1) = 0;  % Initialize reward for this iteration
       
        for i = 1:size(World, 1)
            for j = 1:size(World, 2)
                if World(i, j) == 0
                    old_action = policy(i, j);
                    action_values = zeros(1, 9);
                    for action = 1:9
                        next_state = [i, j] + moves(action, :);
                        if all(next_state >= 1) & all(next_state <= size(World)) & World(next_state(1), next_state(2)) == 0
                            action_values(action) = R(next_state(1), next_state(2)) + gamma * V(next_state(1), next_state(2));
                        else
                            action_values(action) = -Inf;
                        end

                    end
                    % Update policy for current state
                    [max_value, new_action] = max(action_values);
                    policy(i, j) = new_action;
                    if old_action ~= new_action
                        policy_stable = false;
                    end
                end
            end
        end
        % Record iteration time
        iterationTimes = [iterationTimes, toc(startTime)];

        % If policy is stable, return the policy
        if policy_stable
            break;
        end
        
        iterationCount = iterationCount + 1;
    end
    
    elapsedTime = toc(totalStartTime);
end



%{
function [policy, iterationTimes, totalReward, elapsedTime] = policy_iteration_multiple_iterations(R, World, gamma, maxIterations)
    % Initialize policy randomly
    policy = randi([1, 9], size(World));
    
    % Convert obstacles to walls in policy
    policy(World == 1) = 0;
    
    % Define possible moves
    moves = [1, 0; 1, 1; 0, 1; -1, 1; -1, 0; -1, -1; 0, -1; 1, -1; 0, 0]; 
    
    iterationCount = 0;
    iterationTimes = [];  % Array to store the time of each iteration
    totalReward = zeros(1, maxIterations);  % Array to store total reward per iteration
    
    % Measure total elapsed time
    totalStartTime = tic;
    
    % Initialize previousPolicy to something different from the initial policy
    previousPolicy = zeros(size(policy));  % Ensuring it's different to start the iterations

    % Iterate until policy converges or maxIterations reached
    while iterationCount < maxIterations
        % Start timing this iteration
        startTime = tic;
        
        % Use the policy iteration step function to evaluate and improve policy
        [policy, V] = policy_iteration_step(policy, R, World, gamma);
        
        % Calculate total reward for this iteration for visualization
        totalReward(iterationCount + 1) = sum(R(policy == 0)) + sum(sum(R .* (World == 0)));
        
        % Check for policy stability to decide if we should break the loop
        policy_stable = all(policy == previousPolicy);  % Assume previousPolicy is updated at the end of the loop
        
        if policy_stable
            break;
        end
        
        previousPolicy = policy;  % Update the previous policy for the next iteration check
        iterationTimes = [iterationTimes, toc(startTime)];  % Record iteration time
        
        iterationCount = iterationCount + 1;
    end
    
    elapsedTime = toc(totalStartTime);  % Measure total elapsed time for the process
end
%}




maxIterations = 1000
function [bestMove,bestPayoff] = policy_MDP(index,policy,prob)
        %computes the best control action, the (move) that generates the
        %most (payoff) according to the current value function V_hat
        [Iy,Ix] = ind2sub(size(policy),index);
        moves = [1,0; 1,1; 0,1; -1,1; -1,0; -1,-1; 0,-1; 1,-1; 0,0]; 
        bestPayoff = -200; %negative infinity
        probStraight = 0.5;
        for k = [1,3,5,7,2,4,6,8,9]% This order tries straight moves before diagonals %1:size(moves,1) %
            move = [moves(k,1),moves(k,2)];
            if ~prob
                payoff = policy(Iy+move(1),Ix+move(2));
            else    
                if k < 8 %move +45deg of command
                    moveR = [moves(k+1,1),moves(k+1,2)];
                else
                    moveR = [moves(1,1),  moves(1,2)];
                end
                if k>1%move -45deg of command
                    moveL = [moves(k-1,1),moves(k-1,2)]; 
                else
                    moveL = [moves(8,1),  moves(8,2)];
                end
                if isequal(move,[0,0])
                    moveR = [0,0];
                    moveL = [0,0];
                end
                payoff =  probStraight*policy(Iy+move(1), Ix+move(2) )+...
                    (1-probStraight)/2*policy(Iy+moveR(1),Ix+moveR(2))+...
                    (1-probStraight)/2*policy(Iy+moveL(1),Ix+moveL(2));
            end
            
            if payoff > bestPayoff
                bestPayoff = payoff;
                bestMove = move;
            end
        end
    end


function [DX,DY] = DrawPolicy(policy,World,prob)
        % uses arrows to draw the optimal policy according to the Value
        % Funtion approximation policy
        xIND = find(World == 0);
        %subplot(3,2,4)
        fh = figure(); clf
        %colormap(gray)
        set(fh,'units','normalized','outerposition',[0 0 1 1])
        imagesc(policy);
        axis equal
        axis tight
        set(gca,'Xtick',[], 'Ytick',[])
        if prob
            str = 'Policy under probabilistic motion model';
        else
            str = 'Policy under deterministic motion model';
        end
        text(25,-1,str,'HorizontalAlignment','center','FontSize',18);
        [X,Y] = meshgrid(1:size(World,2),1:size(World,1));
        DX = zeros(size(X));
        DY = zeros(size(Y));
        
        for i = 1:numel(xIND)
            [Iy,Ix] = ind2sub(size(policy),xIND(i));
            [bestMove,~] = policy_MDP(xIND(i),policy,prob);
            DX(Iy,Ix) = bestMove(1);
            DY(Iy,Ix) = bestMove(2);
        end
        hold on; hq=quiver(X,Y,DY,DX,0.5,'color',[0,0,0]); hold off
        set(hq,'linewidth',2);
        drawnow
    end
 

%%%%%%%%%%%%%%%%%%%%%%%% correction after error
function [policy, V] = policy_iteration_step(policy, R, World, gamma)
    % One step of policy iteration (policy evaluation + improvement)
    V = zeros(size(World));  % Initialize value function
    V(World == 1) = -Inf;  % Set obstacles to have infinitely negative value

    % Policy evaluation
    for idx = find(World == 0)'  % Loop only over free spaces
        action_values = zeros(1, 9);  % Array to hold values for each action
        for action = 1:9  % Assess each possible action
            action_values(action) = compute_value(idx, action, R, V, gamma, size(World));
        end
        V(idx) = max(action_values);  % Update value function with the max value from possible actions
    end

    % Policy improvement
    for idx = find(World == 0)'  % Loop only over free spaces again
        action_values = zeros(1, 9);  % Reset action values array for each position
        for action = 1:9  % Re-calculate value for each action given updated value function
            action_values(action) = compute_value(idx, action, R, V, gamma, size(World));
        end
        [best_value, best_action] = max(action_values);  % Find the best action
        policy(idx) = best_action;  % Update the policy with the best action
    end
end

function value = compute_value(state, action, R, V, gamma, gridSize)
    % Compute value of taking an action from a given state
    [y, x] = ind2sub(gridSize, state);  % Convert linear index to subscript
    moves = [0, 1; -1, 0; 0, -1; 1, 0; 0, 0; 1, 1; -1, -1; -1, 1; 1, -1];  % Define movements associated with actions
    offset = moves(action, :);
    newY = y + offset(1);
    newX = x + offset(2);
    % Ensure new positions are within grid boundaries
    if newY > 0 && newY <= gridSize(1) && newX > 0 && newX <= gridSize(2)
        nextState = sub2ind(gridSize, newY, newX);
        if World(nextState) == 0  % Ensure the next state is not an obstacle
            value = R(nextState) + gamma * V(nextState);  % Calculate the value
        else
            value = -Inf;  % Penalty for moving into an obstacle
        end
    else
        value = -Inf;  % Penalty for moving out of bounds
    end
end

function offset = action_to_offset(action, gridSize)
    % Map action index to grid offset, e.g., action 5 (stay) to offset (0,0)
    % Define your mapping based on action definitions
    % Example mapping of actions to grid movements
    moves = [0, 1; 1, 0; 0, -1; -1, 0; 0, 0]; % Right, Down, Left, Up, Stay
    if action > length(moves)
        error('Action index out of range');
    end
    offset = moves(action, :);
end
end 