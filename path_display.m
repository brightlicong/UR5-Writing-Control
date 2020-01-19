function [ ] = path_display( char_list,lift_list ,r)
% 显示实际刀路
coVal1 = [251/255 98/255 112/255]; %写字的路径颜色
coVal2 = [36/255 201/255 179/255]; %抬笔的路径颜色
len1 = size(char_list,1);
len2 = size(lift_list,1);
for h=1:len1-1
    begin_node = r.fkine(char_list(h,:));
    end_node = r.fkine(char_list(h+1,:));
    x1 = begin_node(1,4);
    y1 = begin_node(2,4);
    z1 = begin_node(3,4);
    x2 = end_node(1,4);
    y2 = end_node(2,4);
    z2 = end_node(3,4);
    subplot(2,2,1)
    hold on
    plot([x1 x2],[z1 z2],'linewidth',2,'color',coVal1);
%pause(0.02);
    subplot(2,2,2)
    hold on
    plot([y1 y2],[z1 z2],'linewidth',2,'color',coVal1);
    subplot(2,2,[3 4])
    hold on
    plot([x1 x2],[y1 y2],'linewidth',2,'color',coVal1);
end
for h=1:len2-1
    begin_node = r.fkine(lift_list(h,:));
    end_node = r.fkine(lift_list(h+1,:));
    x1 = begin_node(1,4);
    y1 = begin_node(2,4);
    z1 = begin_node(3,4);
    x2 = end_node(1,4);
    y2 = end_node(2,4);
    z2 = end_node(3,4);
    subplot(2,2,1)
    hold on
    plot([x1 x2],[z1 z2],'linewidth',2,'color',coVal2);
    %pause(0.2);
    subplot(2,2,2)
    hold on
    plot([y1 y2],[z1 z2],'linewidth',2,'color',coVal2);
    subplot(2,2,[3 4])
    hold on
    plot([x1 x2],[y1 y2],'linewidth',2,'color',coVal2);
end

end

