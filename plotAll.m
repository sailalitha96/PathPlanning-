x_max= 1000;
y_max=1000;
figure(1)

hold on 

axis([0 x_max 0 y_max])
obstacle1 = [500,150,100,600];
obstacle2= [ 100, 600 , 200 , 200];

%q_goal.point = [200 830];

rectangle('Position',obstacle1,'FaceColor',[0 .5 .5])
rectangle('Position',obstacle2,'FaceColor',[0 .5 .5])
%plot(q_goal.point(1), q_goal.point(2),'*','color', [ 1 0 0 ]);

hold on
for i = 1:2560
    q_current = Nodes(i);
    plot(q_current.point(1),q_current.point(2),'.','color','r','MarkerSize',5);
    while q_current.idx ~= 1
        q_parent = Nodes(q_current.parent);
        line([q_current.point(1), q_parent.point(1)], [q_current.point(2), q_parent.point(2)], 'Color', 'blue', 'LineWidth', 0.2);
        q_current = q_parent;
    end
end