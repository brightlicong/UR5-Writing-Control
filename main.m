tic
sentence_to_write = "���ɧ��";%�ڴ˴�д��UR5������Ҫд�Ļ���
%% ����������Ϣ
%txt�еĵ㶼Ϊ[x y]�ĸ�ʽ�� 
file_path = "dots-information\"+sentence_to_write+"\";
head_file = importdata (file_path+"0.txt");
x_length = head_file(1);%����ĳ��� 
z_length = head_file(2);%����ĸ߶�
img_num = head_file(3);%�ؼ�����

%% �����Ϣ����
pen_length = 0.1; %�ʵĳ���
lift_dis = 0.05; %̧�ʵľ���
original_display_mode = 0; %�Ƿ���ʾ��·Ԥ��ͼ
%% 
r1 = creat_ur5('left',transl(0,-0.544,0)*trotz(pi));
r2 = creat_ur5('right',transl(0,0.544,0));
Qlist = [];
dots_list = cell(img_num,1); %�洢���еĹؼ�����Ϣ 
head_tail_inf = zeros(img_num,5);

%% ����д˳���������
%������[0 0]��ʼ���Ծ���Ϊ��׼Ѱ������ĵ㡣
this_point = [0 0];
for i= 1:img_num
    file_name = file_path+num2str(head_index)+'.txt';
    file = importdata (file_name);
    head_tail_inf(i,1) = i;
    head_tail_inf(i,2:3) = file(1,:);
    head_tail_inf(i,4:5) = file(end,:);
end
for i= 1:img_num
    [head_index,head_dis] = find_closed_point(this_point,head_tail_inf(:,[1 2 3]));
    [tail_index,tail_dis] = find_closed_point(this_point,head_tail_inf(:,[1 4 5]));
    display([i head_index head_dis  tail_index tail_dis])
    if head_dis < tail_dis
        head_delet_index = find(head_tail_inf(:,1)==head_index);
        this_point = head_tail_inf(head_delet_index,[2 3]);
        file_name = file_path+num2str(head_index)+'.txt';
        file = importdata (file_name);
        dots_list{i,1}= file;        
        head_tail_inf(head_delet_index,:)=[];
        display([i head_index head_delet_index]);
    else
        tail_delet_index = find(head_tail_inf(:,1)==tail_index);
        this_point = head_tail_inf(tail_delet_index,[4 5]);
        file_name = file_path+num2str(tail_index)+'.txt';
        file = importdata (file_name);
        dots_list{i,1}= flipud(file);%���·�ת����
        head_tail_inf(tail_delet_index,:)=[];
        display([i tail_index tail_delet_index]);
    end   
end


%% ����ؽڽ�
for i= 1:img_num
    coVal = [rand rand rand];
    %дһ��
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
        %�Խ�����в���

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
        a = ur5_ikine(r2,trajectory);
        Qlist = [Qlist;a a];
    end
    %̧�����
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
        end
    end
end
%% �Գְ�����˹ؽڽǽ����޸�
Q_num = size(Qlist,1);
sub_joints = [0,-pi/2,0,-pi/2,0,0];
for m = 1:Q_num
    Qlist(m,1:6) = sub_joints;
end
%% �ؽڽ�ת��Ϊtxt�ļ�
writeQlist(Qlist);
%% �������
test_plot(Qlist,r2,r1);
toc