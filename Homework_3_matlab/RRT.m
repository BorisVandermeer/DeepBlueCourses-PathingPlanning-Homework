%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clear all; close all;
x_I=1; y_I=1;           % ���ó�ʼ��
x_G=700; y_G=700;       % ����Ŀ���
Thr=50;                 %����Ŀ�����ֵ
Delta= 30;              % ������չ����
%% ������ʼ��
T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;     %
T.v(2).x = 2;
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
count=1;
for iter = 1:3000
    %Step 1: �ڵ�ͼ���������һ����x_rand
    %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
    % x_rand = [floor(rand(1)*xL),floor(rand(1)*yL)];
    x_rand = randi([1,yL],1,2);
    %Step 2: ���������������ҵ�����ڽ���x_near 
    %��ʾ��x_near�Ѿ�����T��
    dist = realmax();
    idx_tmp = 1;
    for i = 1:count
        dist_tmp = sqrt((T.v(i).x-x_rand(1))^2+(T.v(i).y-x_rand(2))^2);
        if dist_tmp<dist
            dist = dist_tmp;
            idx_tmp = i;
        end
    end
    x_near=[T.v(idx_tmp).x,T.v(idx_tmp).y];
    
    %Step 3: ��չ�õ�x_new�ڵ�
    %��ʾ��ע��ʹ����չ����Delta
    x_new=x_near + Delta * [x_rand(1)-x_near(1),x_rand(2)-x_near(2)] /...
                   sqrt((x_rand(1)-x_near(1))^2+(x_rand(2)-x_near(2))^2);
    %���ڵ��Ƿ���collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
        continue;
    end
        
    %Step 4: ��x_new������T 
    %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
    count=count+1;
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = T.v(idx_tmp).dist+dist;
    T.v(count).indPrev = idx_tmp;
    %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
    %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
    if(sqrt((x_new(1)-x_G)^2+(x_new(2)-y_G)^2)<Thr)
        break;
    end
    %Step 6:��x_near��x_new֮���·��������
    %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
    %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r')
    hold on;
   
    pause(0.01); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
end
%% ·���Ѿ��ҵ��������ѯ
if iter < 2000
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % �յ����·��
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % ���յ���ݵ����
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


