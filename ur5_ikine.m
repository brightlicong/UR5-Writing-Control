function [ qtraj ] = ur5_ikine( ur5 , ctraj , opt )
%逆解一组路径点，并选择关节角变化最小的解，输入4*4*m齐次矩阵，输出m*6关节角。

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
    
    %根据opt取一个初值/默认
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
        
        %获得关节角变化最小的一组解
        delta = zeros(8,6);
        relative_delta = zeros(8,6);
        
        for j = 1:8
            
            %相减并取绝对值后，最佳的一组解差最接近2pi或0
            delta(j,:) = abs( cell2mat( available_q(j) ) - qtraj(i - 1,:) );
            for k = 1:6
                
                if delta(j,k) >= pi
                    
                    %对于差大于pi的，用差除以-pi的余数的绝对值作为其变化量
                    %                     modded_delta(j,k) = abs( mod( delta(j,k) , -pi ) );
                    relative_delta(j,k) = 2 * pi - delta(j,k);
                    
                else
                    
                    %对于差小于pi的，用差除以pi的余数的绝对值作为其变化量
                    %                     modded_delta(j,k) = abs( mod( delta(j,k) , pi ) );
                    relative_delta(j,k) = delta(j,k);
                    
                end
                
            end
            
        end
        
        %         disp(modded_delta);
        
        %找到每组解最大的变化量，并从这些最大的变化量中挑出最小的，所得到的即为变化最小的一组解
        [~,best_solution_index] = min(max(relative_delta,[],2));
        qtraj(i,:) = cell2mat( available_q(best_solution_index) );
        
    end
    
end

