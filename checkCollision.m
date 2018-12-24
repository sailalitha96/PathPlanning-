function isCollided = checkCollision(obstacle, p1, p2)
  
  x_min = obstacle(1);  y_min = obstacle(2); 
  x_max = x_min + obstacle(3); y_max = y_min + obstacle(4);
  
  % define the four vertex of the rectangle
  v1 = [x_min y_min]; v2 = [x_min y_max]; v3 = [x_max y_max]; v4 = [x_max y_min];
  
  isCollided = false;
  flag(1) = lineIntersect(v1,v2,p1,p2);
  flag(2) = lineIntersect(v2,v3,p1,p2);
  flag(3) = lineIntersect(v3,v4,p1,p2);
  flag(4) = lineIntersect(v4,v1,p1,p2);
  
  if any(flag == 1)
      isCollided = true;
  end

end
  
function intersect = lineIntersect(p1,p2,p3,p4)
  
  x1 = p1(1); x2 = p2(1); x3 = p3(1); x4 = p4(1);
  y1 = p1(2); y2 = p2(2); y3 = p3(2); y4 = p4(2);

  x=[x1 x2 x3 x4];
  y=[y1 y2 y3 y4];
  dt1=det([1,1,1;x(1),x(2),x(3);y(1),y(2),y(3)])*det([1,1,1;x(1),x(2),x(4);y(1),y(2),y(4)]);
  dt2=det([1,1,1;x(1),x(3),x(4);y(1),y(3),y(4)])*det([1,1,1;x(2),x(3),x(4);y(2),y(3),y(4)]);

  if(dt1<=0 & dt2<=0)
    intersect=1;         %If lines intesect
  else
    intersect=0;
  end
end