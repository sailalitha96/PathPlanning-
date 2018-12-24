function collision = checkAllObstacles(obstacles, p1, p2)
             collision = false;
             for i=1:size(obstacles,1)
                if checkCollision(obstacles(i,:),p1,p2)
                  collision = true;
                  return
                end
             end
end