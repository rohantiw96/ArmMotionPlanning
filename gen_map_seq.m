function [envmap] = gen_map_seq(mapsize, obstaclesize, actionvec, startx, starty)

envmap = zeros(mapsize, mapsize, length(actionvec));
curr_pos = [startx, starty];

for idx=1:length(actionvec)
    obstacle_end_x = curr_pos(1) + obstaclesize(1)-1;
    obstacle_end_y = curr_pos(2) + obstaclesize(2)-1;
    
    if(obstacle_end_x > mapsize)
        obstacle_end_x = mapsize;
    end
    if(obstacle_end_y > mapsize)
        obstacle_end_y = mapsize;
    end
    
    envmap(curr_pos(2):obstacle_end_y, curr_pos(1):obstacle_end_x, idx) = 1;
    
    curr_pos = curr_pos + actionvec(idx,:);
end

end