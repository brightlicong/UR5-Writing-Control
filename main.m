tic
sentence_to_write = "你好骚啊";%在此处写明UR5机器人要写的话。
%% 导入总体信息
%txt中的点都为[x y]的格式。 
file_path = "dots-information\"+sentence_to_write+"\";
head_file = importdata (file_path+"0.txt");
x_length = head_file(1);%画板的长度 
z_length = head_file(2);%画板的高度
img_num = head_file(3);%关键点数  
%% 相关信息设置
pen_length = 0.1; %笔的长度
lift_dis = 0.05; %抬笔的距离
original_display_mode = 1; %是否显示理想预览图
path_disply_mode = 1;%是否显示实际刀路
coVal = [rand rand rand];

%% 必要对象的声明 
r1 = creat_ur5('left',transl(0,-0.544,0)*trotz(pi)); %左臂机器人
r2 = creat_ur5('right',transl(0,0.544,0)); %右臂机器人
Qlist = [];
dots_list = cell(img_num,1); %存储所有的关键点信息 
head_tail_inf = zeros(img_num,5);%储存每个笔画的头与尾的坐标
%% 导入关键点信息
for i= 1:img_num
    file_name = file_path+num2str(i)+'.txt';
    file = importdata (file_name);
    head_tail_inf(i,1) = i;
    head_tail_inf(i,2:3) = file(1,:);
    head_tail_inf(i,4:5) = file(end,:);
end
%% 对书写顺序进行排序
%贪心算法
%从坐标[0 0]开始，以距离为标准寻找最近的点。
this_point = [0 0];

for i= 1:img_num
    [head_index,head_dis] = find_closed_point(this_point,head_tail_inf(:,[1 2 3]));
    [tail_index,tail_dis] = find_closed_point(this_point,head_tail_inf(:,[1 4 5]));
    %display([i head_index head_dis  tail_index tail_dis])
    if head_dis < tail_dis
        head_delet_index = find(head_tail_inf(:,1)==head_index);
        this_point = head_tail_inf(head_delet_index,[2 3]);
        file_name = file_path+num2str(head_index)+'.txt';
        file = importdata (file_name);
        dots_list{i,1}= file;        
        head_tail_inf(head_delet_index,:)=[];
        %display([i head_index head_delet_index]);
    else
        tail_delet_index = find(head_tail_inf(:,1)==tail_index);
        this_point = head_tail_inf(tail_delet_index,[4 5]);
        file_name = file_path+num2str(tail_index)+'.txt';
        file = importdata (file_name);
        dots_list{i,1}= flipud(file);%上下翻转矩阵
        head_tail_inf(tail_delet_index,:)=[];
        %display([i tail_index tail_delet_index]);
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
        %显示理想结果
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
                    figure(1);
                    plot([x1 x2],[z1 z2],'linewidth',2,'color',coVal);
                    %pause(0.1)
                end
            end
        end        
        a = ur5_ikine(r2,trajectory);
        Qlist = [Qlist;a a];
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
            a = ur5_ikine(r2,trajectory);
            Qlist = [Qlist;a a];
            lift_list = [lift_list;a];
        end
    end
    if path_disply_mode == 1    
    %实际刀路测试
    disp(i);
    path_display( char_list,lift_list,r2 );  
    end
end
%% 对持板机器人关节角进行修改
Q_num = size(Qlist,1);
sub_joints = [0,-pi/2,0,-pi/2,0,0];
for m = 1:Q_num
    Qlist(m,1:6) = sub_joints;
end
%% 关节角转化为txt文件
writeQlist(Qlist);
%% 结果测试
%test_plot(Qlist,r2,r1);
toc