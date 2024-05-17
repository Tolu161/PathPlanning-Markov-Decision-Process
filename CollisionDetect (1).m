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

