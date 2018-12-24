
%% RRT*
% Authors: Yuwei Wang , Sailalitha Gollapudi , Mohit Patel 

%% Initialize for plotting 
clear
x_max= 1000;
y_max=1000;
figure(1)
hold on

axis([0 x_max 0 y_max])
scale = 1000/3;

% Map 1: Maze
% obstacle1 = [0.5,0.6,0.3,2.4].*scale;
% obstacle2 = [1.3,0,0.3,2.4].*scale;
% obstacle3 = [2.1,0.6,0.3,2.4].*scale;
% obstacles = [obstacle1; obstacle2; obstacle3];

% Map 2: Concave
obstacle1 = [0.9, 1.2, 0.2, 0.5].*scale;
obstacle2 = [1.1, 1.5, 0.8, 0.2].*scale;
obstacle3 = [1.9, 0.7, 0.2, 1].*scale;
obstacle4 = [1.5, 0.7, 0.4, 0.2].*scale;
obstacle5 = [2.2, 2, 0.2, 0.4].*scale;
obstacle6 = [2.4, 2, 0.2, 0.2].*scale;
obstacles = [obstacle1; obstacle2; obstacle3; obstacle4; obstacle5; obstacle6];

%plot obstacles
for i=1:size(obstacles,1)
   rectangle('Position',obstacles(i,:),'FaceColor',[0 .5 .5])
end

%plot goal
q_goal.point = [2.5 2.6] .* scale;
%% Initialize for q_start and the tree 'Nodes'
q_start.point= [0.5 0.4] .* scale;
q_start.cost = 0;
q_start.parent = 1;
q_start.child = 1;
q_start.idx = 1;
Nodes = q_start;

q_nearGoal = [];
dthres= 10 ;
r = 50;

samples= 5000;
hold on
%% Main loop for growing the tree
for s = 1:samples
    s
    p_rand = [round(rand(1)*x_max) round(rand(1)*y_max)];         % sample a point
    
    min_dist = 10000;
    for j = 1: length(Nodes)
        if norm(Nodes(j).point - p_rand) < min_dist
            min_dist = norm(Nodes(j).point - p_rand);
            q_nearest = Nodes(j);
        end
    end
 
    distance = norm(p_rand - q_nearest.point);
    q_new.point= steering(p_rand,q_nearest.point,distance,dthres);
    
    if ~checkAllObstacles(obstacles, q_new.point,q_nearest.point)
        Q_near = [];
        for i = 1:length(Nodes)
            if norm(Nodes(i).point - q_new.point) < r
                Q_near = [Q_near; Nodes(i)];
            end
        end
        q_new.idx = length(Nodes) + 1;
        q_new.child = q_new.idx;
              
        min_idx = q_nearest.idx;
        q_min = q_nearest;
        c_min = q_nearest.cost + norm(q_nearest.point - q_new.point);
        for j = 1:length(Q_near)
            if ~checkAllObstacles(obstacles, q_new.point,Q_near(j).point) ...
               && Q_near(j).cost + norm(Q_near(j).point - q_new.point) < c_min
                q_min = Q_near(j);
                c_min = Q_near(j).cost + norm(Q_near(j).point - q_new.point);
                min_idx = Q_near(j).idx;
            end
        end
        
        % connect q_new to q_min
        q_new.parent = min_idx;     
        Nodes(min_idx).child = [Nodes(min_idx).child;q_new.idx];  
        q_new.cost = c_min;
     %   line([q_new.point(1), q_min.point(1)], [q_new.point(2), q_min.point(2)], 'Color', 'black', 'LineWidth', 0.5);
        
        Nodes = [Nodes;q_new];             % add q_new to the Tree
        
        %% Rewiring
        for j = 1:length(Q_near)
            Q_near(j) = Nodes(Q_near(j).idx);
            if ~checkAllObstacles(obstacles, q_new.point,Q_near(j).point)...
               && q_new.cost + norm(q_new.point - Q_near(j).point) < Q_near(j).cost               
    %            line([Q_near(j).point(1), old_parent.point(1)], [Q_near(j).point(2), old_parent.point(2)], 'Color', 'white', 'LineWidth', 0.5);
                 old_parent = Q_near(j).parent;
                 Nodes(old_parent).child = remove(Nodes(old_parent).child, Q_near(j).idx);   
                  
                 Q_near(j).parent = q_new.idx;
                 q_new.child = [q_new.child; Q_near(j).idx];
                  
                 Nodes(Q_near(j).idx) = Q_near(j);
                 Nodes(q_new.idx) = q_new;

                 % updating costs for Q_near(j) and its children
                 Nodes = updateChildrenCost(Nodes(q_new.idx), q_new, Nodes);
      %          line([q_new.point(1), Q_near(j).point(1)], [q_new.point(2), Q_near(j).point(2)], 'Color', 'black', 'LineWidth', 0.5);
%                  
            end       
        end
       
       %% check if goal is reached     
        if norm(q_new.point - q_goal.point) < 30
           fprintf(' you reached the goal');
           q_nearGoal = [q_nearGoal;q_new];
        end
    end

end

%% plot the entire tree

for i = 1:length(Nodes)
    q_current = Nodes(i);
    plot(q_current.point(1),q_current.point(2),'.','color','r','MarkerSize',5);
    while q_current.idx ~= 1
        q_parent = Nodes(q_current.parent);
        line([q_current.point(1), q_parent.point(1)], [q_current.point(2), q_parent.point(2)], 'Color', 'g', 'LineWidth', 0.2);
        q_current = q_parent;
    end
end

plot(q_start.point(1), q_start.point(2),'*','color', 'b');
plot(q_goal.point(1), q_goal.point(2),'*','color', [ 1 0 0 ]);

%% Recover and plot the cheapest path
if ~isempty(q_nearGoal)
    min_cost = 10000;
    q_Goal_min = q_nearGoal(1);
    for i = 1:length(q_nearGoal)
        if q_nearGoal(i).cost < min_cost
            min_cost = q_nearGoal(i).cost;
            q_Goal_min = q_nearGoal(i);
        end
    end

    q_current = q_Goal_min;
    while q_current.idx ~= 1
        q_parent = Nodes(q_current.parent);
        line([q_current.point(1), q_parent.point(1)], [q_current.point(2), q_parent.point(2)], 'Color', 'r', 'LineWidth', 1.5);
        q_current = q_parent;
    end
end

        
        
        
        