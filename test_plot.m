function [] = test_plot(Q,main_robot,sub_robot)
len = size(Q,1);
show_robot_mode = 0;
for i=1:len-1
    if show_robot_mode == 1
     main_robot.plot(Q(i,7:12));
     hold on
     sub_robot.plot(Q(i,1:6));
     hold off 
    end
    begin_node = main_robot.fkine(Q(i,7:12));
    end_node = main_robot.fkine(Q(i+1,7:12));
    x1 = begin_node(1,4);
    y1 = begin_node(2,4);
    z1 = begin_node(3,4);
    x2 = end_node(1,4);
    y2 = end_node(2,4);
    z2 = end_node(3,4);
    if y1 < 0.11
        subplot(2,2,1)
        hold on
        plot([x1 x2],[z1 z2],'linewidth',2,'color','m');
        pause(0.2);
    end
    subplot(2,2,2)
    hold on
    plot([y1 y2],[z1 z2],'linewidth',2,'color','m');
    subplot(2,2,[3 4])
    hold on
    plot([x1 x2],[y1 y2],'linewidth',2,'color','m');

end


end

