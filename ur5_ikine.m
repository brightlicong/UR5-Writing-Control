function [ qtraj ] = ur5_ikine( ur5 , ctraj , opt )
%���һ��·���㣬��ѡ��ؽڽǱ仯��С�Ľ⣬����4*4*m��ξ������m*6�ؽڽǡ�

if(~ishomog(ctraj))
    disp('ctraj is not homogeneous transformation!');
    return;
end

size_ctraj = size( ctraj );
Tq_quantity = size_ctraj(3);

qtraj = zeros(Tq_quantity,6);

% 1:rd+
% 2:ru+
% 3:rd-
% 4:ru-
% 5:lu+
% 6:ld+
% 7:lu-
% 8:ld-

for i = 1:Tq_quantity
    
    %disp(i);
    %     disp(qtraj);
    available_q = ur5_ikine_single( ur5 , ctraj(:,:,i) );
    
    %����optȡһ����ֵ/Ĭ��
    if i==1
        
        if nargin == 3
            
            if(strlength(opt)~=3)
                disp('opt must contain 3 characters!')
                return;
            end
            
            switch opt
                case 'rd+'
                    qtraj(i,:) = cell2mat( available_q(1) );
                case 'ru+'
                    qtraj(i,:) = cell2mat( available_q(2) );
                case 'rd-'
                    qtraj(i,:) = cell2mat( available_q(3) );
                case 'ru-'
                    qtraj(i,:) = cell2mat( available_q(4) );
                case 'lu+'
                    qtraj(i,:) = cell2mat( available_q(5) );
                case 'ld+'
                    qtraj(i,:) = cell2mat( available_q(6) );
                case 'lu-'
                    qtraj(i,:) = cell2mat( available_q(7) );
                case 'ld-'
                    qtraj(i,:) = cell2mat( available_q(8) );
                otherwise
                    disp('opt must be one of eight options!')
                    return;
            end
            
        elseif nargin == 2
            
            qtraj(i,:) = cell2mat( available_q(1) );
            
        end
        
    else
        
        %��ùؽڽǱ仯��С��һ���
        delta = zeros(8,6);
        relative_delta = zeros(8,6);
        
        for j = 1:8
            
            %�����ȡ����ֵ����ѵ�һ������ӽ�2pi��0
            delta(j,:) = abs( cell2mat( available_q(j) ) - qtraj(i - 1,:) );
            for k = 1:6
                
                if delta(j,k) >= pi
                    
                    %���ڲ����pi�ģ��ò����-pi�������ľ���ֵ��Ϊ��仯��
                    %                     modded_delta(j,k) = abs( mod( delta(j,k) , -pi ) );
                    relative_delta(j,k) = 2 * pi - delta(j,k);
                    
                else
                    
                    %���ڲ�С��pi�ģ��ò����pi�������ľ���ֵ��Ϊ��仯��
                    %                     modded_delta(j,k) = abs( mod( delta(j,k) , pi ) );
                    relative_delta(j,k) = delta(j,k);
                    
                end
                
            end
            
        end
        
        %         disp(modded_delta);
        
        %�ҵ�ÿ������ı仯����������Щ���ı仯����������С�ģ����õ��ļ�Ϊ�仯��С��һ���
        [~,best_solution_index] = min(max(relative_delta,[],2));
        qtraj(i,:) = cell2mat( available_q(best_solution_index) );
        
    end
    
end

