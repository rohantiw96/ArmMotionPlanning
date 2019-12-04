function [envmap] = gen_map_seq_infl(mapsize, obstaclesize, actionvec, startx, starty,inflation)

envmap = zeros(mapsize, mapsize, length(actionvec));
curr_pos = [startx-inflation, starty-inflation];
obstaclesize= obstaclesize + 2*inflation*ones(1,2);
for idx=1:length(actionvec)
    obstacle_end_x = curr_pos(1) + obstaclesize(1)-1+2*inflation;
    obstacle_end_y = curr_pos(2) + obstaclesize(2)-1+2*inflation;
    
    % check if object goes off the map
    rollover_start = curr_pos;
    rollover_x_end = obstacle_end_x;
    rollover_y_end = obstacle_end_y;
    if(obstacle_end_x > mapsize)
        rollover_start(1) = 1;
        rollover_x_end = mod(obstacle_end_x, mapsize);
        obstacle_end_x = mapsize;
    end
    if(obstacle_end_y > mapsize)
        rollover_start(2) = 1;
        rollover_y_end = mod(obstacle_end_y, mapsize);
        obstacle_end_y = mapsize;
    end
    
    % plot
    curr_pos(2)
    obstacle_end_y
    curr_pos(1)
    obstacle_end_x
    idx
    envmap(curr_pos(2):obstacle_end_y, curr_pos(1):obstacle_end_x, idx) = 1;
    envmap(rollover_start(2):rollover_y_end, rollover_start(1):rollover_x_end, idx) = 1;
    
    % update current position of obstacle
    curr_pos = curr_pos + actionvec(idx,:);
    
    if(any(curr_pos <= 0))  % if position goes off at the 0-border:
        jdx = find(curr_pos <= 0);
        for i=1:length(jdx)
            curr_pos(jdx(i)) = mapsize;
        end
        
    elseif(any(curr_pos > mapsize)) % if position goes off at the end
        jdx = find(curr_pos > mapsize);
        for i=1:length(jdx)
            curr_pos(jdx(i)) = 1;
        end
    end
    
end

end