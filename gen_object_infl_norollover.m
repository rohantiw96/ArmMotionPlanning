function [map] = gen_object_infl_norollover(map, obstaclesize, actionvec, startx, starty,inflation)

mapsize = size(map,1);
curr_pos = [startx-inflation, starty-inflation];
obstaclesize= obstaclesize + 2*inflation*ones(1,2);
for idx=1:length(actionvec)
    obstacle_end_x = curr_pos(1) + obstaclesize(1)-1+2*inflation;
    obstacle_end_y = curr_pos(2) + obstaclesize(2)-1+2*inflation;
    
    % check if object goes off the map
    if(obstacle_end_x > mapsize)
        obstacle_end_x = mapsize;
    end
    if(obstacle_end_y > mapsize)
        obstacle_end_y = mapsize;
    end
    
    % plot
    map(curr_pos(2):obstacle_end_y, curr_pos(1):obstacle_end_x, idx) = 1;
    
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