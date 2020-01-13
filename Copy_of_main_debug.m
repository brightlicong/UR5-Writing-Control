tic
%% 文件说明
% 对机器人运动结果进行测试
%% 相关信息设置
pen_length = 0.1;
lift_dis = 0.05;
strokes_sorting_mode = 1;
original_display_mode = 0;
r1 = creat_ur5('left',transl(0,-0.544,0)*trotz(pi));
r2 = creat_ur5('right',transl(0,0.544,0));
coVal = [rand rand rand];
coVala = [rand rand rand];
coValb = [rand rand rand];
%% 导入总体信息
%所有数据都是先x轴再z轴
head_file_name = "dots_information\0.txt";
head_file = importdata (head_file_name);
x_length = head_file(1);
z_length = head_file(2);
img_num = head_file(3);%为了测试可做修改修改
head_tail_inf = zeros(img_num,5);
%% 导入笔画信息
if strokes_sorting_mode == 1
    for i= 1:img_num
        file_name = ['dots_information\',num2str(i),'.txt'];
        file = importdata (file_name);
        head_tail_inf(i,1) = i;
        head_tail_inf(i,2:3) = file(1,:);
        head_tail_inf(i,4:5) = file(end,:);
    end

    dots_list = cell(img_num,1); %存储所有的关键点信息
    this_point = [0 0];

   for i= 1:img_num
    [head_index,head_dis] = find_closed_point(this_point,head_tail_inf(:,[1 2 3]));
    [tail_index,tail_dis] = find_closed_point(this_point,head_tail_inf(:,[1 4 5]));

        if head_dis < tail_dis
            head_delet_index = find(head_tail_inf(:,1)==head_index);
            this_point = head_tail_inf(head_delet_index,[2 3]);
            file_name = ['dots_information\',num2str(head_index),'.txt'];
            file = importdata (file_name);
            dots_list{i,1}= file;        
            head_tail_inf(head_delet_index,:)=[];

        else
            tail_delet_index = find(head_tail_inf(:,1)==tail_index);
            this_point = head_tail_inf(tail_delet_index,[4 5]);
            file_name = ['dots_information\',num2str(tail_index),'.txt'];
            file = importdata (file_name);
            dots_list{i,1}= flipud(file);%上下翻转矩阵
            head_tail_inf(tail_delet_index,:)=[];

        end   
    end
else
    dots_list = cell(img_num,1); %存储所有的关键点信息
    for i = 1:img_num
        file_name = ['dots_information\',num2str(i),'.txt'];
        file = importdata (file_name);
        dots_list{i,1}= file;        
    end
end



%% 计算关节角
for i= 1:img_num
    char_list = [];
    lift_list = [];

    %写一笔
    length = size(dots_list{i,1},1);
    T = cell(length-1,2);
    for j = 1:length - 1
        T{j,1} = transl(0,pen_length,0)*cordinate2T(dots_list{i,1}(j,1),dots_list{i,1}(j,2),x_length,z_length)*trotx(pi/2);
        T{j,2} = transl(0,pen_length,0)*cordinate2T(dots_list{i,1}(j+1,1),dots_list{i,1}(j+1,2),x_length,z_length)*trotx(pi/2);
    end
    for j = 1:length-1
        steps = get_steps_by_dis(T{j,1}(1,4),T{j,1}(3,4),T{j,2}(1,4),T{j,2}(3,4));
        trajectory =ctraj(T{j,1},T{j,2},steps);
        if original_display_mode == 1
        %对结果进行测试
            for m = 1:steps
                if m >1 
                    hold on;
                    begin_node = trajectory(:,:,m-1);
                    end_node = trajectory(:,:,m);
                    x1 = begin_node(1,4);
                    y1 = begin_node(2,4);
                    z1 = begin_node(3,4);
                    x2 = end_node(1,4);
                    y2 = end_node(2,4);
                    z2 = end_node(3,4);
                    plot([x1 x2],[z1 z2],'linewidth',2,'color',coVal);
                    %plot3([x1 x2],[y1 y2],[z1 z2],'color','k');
                    pause(0.2)
                end
            end
        end        
        a = ur5_ikine_v2(r2,trajectory);
        char_list = [char_list;a];
    end
    %抬笔落笔
    if i > 1
        T = cell(3,2);
        up_point = dots_list{i-1,1}(end,:);
        down_point = dots_list{i,1}(1,:);
        lala1 = transl(0,pen_length,0)*cordinate2T(up_point(1,1),up_point(1,2),x_length,z_length)*trotx(pi/2);
        lala2 = transl(0,lift_dis,0)*transl(0,pen_length,0)*cordinate2T(up_point(1,1),up_point(1,2),x_length,z_length)*trotx(pi/2);
        lala3 = transl(0,lift_dis,0)*transl(0,pen_length,0)*cordinate2T(down_point(1,1),down_point(1,2),x_length,z_length)*trotx(pi/2);
        lala4 = transl(0,pen_length,0)*cordinate2T(down_point(1,1),down_point(1,2),x_length,z_length)*trotx(pi/2);
        T{1,1} = lala1;
        T{1,2} = lala2;
        T{2,1} = lala2;
        T{2,2} = lala3;
        T{3,1} = lala3;
        T{3,2} = lala4;
        for j = 1:3
            steps = get_steps_by_dis(T{j,1}(1,4),T{j,1}(3,4),T{j,2}(1,4),T{j,2}(3,4));
            trajectory =ctraj(T{j,1},T{j,2},steps);
            a = ur5_ikine_v2(r2,trajectory);
            lift_list = [lift_list;a];
        end
    end
%最后的测试
    len1 = size(char_list,1);
    len2 = size(lift_list,1);
    for h=1:len1-1

        begin_node = r2.fkine(char_list(h,:));
        end_node = r2.fkine(char_list(h+1,:));
        x1 = begin_node(1,4);
        y1 = begin_node(2,4);
        z1 = begin_node(3,4);
        x2 = end_node(1,4);
        y2 = end_node(2,4);
        z2 = end_node(3,4);
        subplot(2,2,1)
        hold on
        plot([x1 x2],[z1 z2],'linewidth',2,'color',coVala);
        %pause(0.02);
        subplot(2,2,2)
        hold on
        plot([y1 y2],[z1 z2],'linewidth',2,'color',coVala);
        subplot(2,2,[3 4])
        hold on
        plot([x1 x2],[y1 y2],'linewidth',2,'color',coVala);
    end
     for h=1:len2-1

        begin_node = r2.fkine(lift_list(h,:));
        end_node = r2.fkine(lift_list(h+1,:));
        x1 = begin_node(1,4);
        y1 = begin_node(2,4);
        z1 = begin_node(3,4);
        x2 = end_node(1,4);
        y2 = end_node(2,4);
        z2 = end_node(3,4);
        subplot(2,2,1)
        hold on
        plot([x1 x2],[z1 z2],'linewidth',2,'color',coValb);
        %pause(0.2);
        subplot(2,2,2)
        hold on
        plot([y1 y2],[z1 z2],'linewidth',2,'color',coValb);
        subplot(2,2,[3 4])
        hold on
        plot([x1 x2],[y1 y2],'linewidth',2,'color',coValb);
    end
    
end
toc