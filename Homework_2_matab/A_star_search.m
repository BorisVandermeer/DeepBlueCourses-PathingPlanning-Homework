function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];
    %Parent LIST STRUCTURE
    %--------------
    %X val | Y val |Parent X val | Parent Y val|
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    Parent = [];
    Parent_CNT = 0;

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(OPEN_COUNT) %you have to dicide the Conditions for while loop exit 
        %如果列表为空，那么就认为到不了，NoPath保持为1
        %找到最小的那一个
        min_h = OPEN(1,5);
        min_h_idx = 1;
        for i= 1:OPEN_COUNT
            if(OPEN(i,5)<min_h) 
                min_h_idx = i;
                min_h = OPEN(i,5);
            end
        end
        cur_x = OPEN(min_h_idx,1);
        cur_y = OPEN(min_h_idx,2);
        Parent_CNT = Parent_CNT+1;
        Parent(Parent_CNT,:) = [cur_x,cur_y,OPEN(min_h_idx,3),OPEN(min_h_idx,4)];
        CLOSED_COUNT = CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,:) = [cur_x,cur_y];
        if(cur_x==xTarget&&cur_y==yTarget)
            NoPath = 0;
            break;
        end
        BLOCKER = [0,0,0,0,0,0,0,0];
        for i = 1:CLOSED_COUNT
            if(cur_x==1)
                BLOCKER([1,4,6]) = [1,1,1];
            end
            if(cur_x==MAX_X)
                BLOCKER([3,5,8]) = [1,1,1];
            end
            if(cur_y==1)
                BLOCKER([1,2,3]) = [1,1,1];
            end
            if(cur_y==MAX_Y)
                BLOCKER([6,7,8]) = [1,1,1];
            end
            if(CLOSED(i,1)==cur_x-1)
                if (CLOSED(i,2)==cur_y-1)
                    BLOCKER(1) = 1;
                elseif (CLOSED(i,2)==cur_y)
                    BLOCKER(4) = 1;
                elseif(CLOSED(i,2)==cur_y+1)
                    BLOCKER(6) =1;
                end
            end
            if(CLOSED(i,1)==cur_x)
                if(CLOSED(i,2)==cur_y-1)
                    BLOCKER(2) = 1;
                elseif(CLOSED(i,2)==cur_y+1)
                    BLOCKER(7) = 1;
                end
            end
            if(CLOSED(i,1)==cur_x+1)
                if (CLOSED(i,2)==cur_y-1)
                    BLOCKER(3) = 1;
                elseif (CLOSED(i,2)==cur_y)
                    BLOCKER(5) = 1;
                elseif(CLOSED(i,2)==cur_y+1)
                    BLOCKER(8) = 1;
                end
            end
        end
        TOPUSH = [cur_x-1,cur_y-1;cur_x,cur_y-1;cur_x+1,cur_y-1;
                  cur_x-1,cur_y;                cur_x+1,cur_y;
                  cur_x-1,cur_y+1;cur_x,cur_y+1;cur_x+1,cur_y+1];
        for i = [1,3,6,8]
            if(BLOCKER(i)==0)
                InOpenFlag = 0;
                Index = 0;
                for j = 1:OPEN_COUNT
                    if(OPEN(j,1)==TOPUSH(i,1))&&(OPEN(j,2)==TOPUSH(i,2))
                        Index = j;
                        InOpenFlag = 1;
                        break;
                    end
                end
                if(InOpenFlag == 1)
                    if (OPEN(min_h_idx,6)+1<OPEN(Index,6))
                        OPEN(Index,6) = OPEN(min_h_idx,6)+1;
                    end
                else
                    OPEN_COUNT = OPEN_COUNT+1;
                    OPEN(OPEN_COUNT,:) = [TOPUSH(i,1),TOPUSH(i,2),cur_x,cur_y, ...
                                          OPEN(min_h_idx,6)+sqrt(2)+distance(TOPUSH(i,1),TOPUSH(i,2),xTarget,yTarget), ...
                                          OPEN(min_h_idx,6)+sqrt(2),distance(TOPUSH(i,1),TOPUSH(i,2),xTarget,yTarget)];
                end
            end
        end
        for i = [2,4,5,7]
            if(BLOCKER(i)==0)
                InOpenFlag = 0;
                Index = 0;
                for j = 1:OPEN_COUNT
                    if(OPEN(j,1)==TOPUSH(i,1))&&(OPEN(j,2)==TOPUSH(i,2))
                        Index = j;
                        InOpenFlag = 1;
                        break;
                    end
                end
                if(InOpenFlag == 1)
                    if (OPEN(min_h_idx,6)+1<OPEN(Index,6))
                        OPEN(Index,6) = OPEN(min_h_idx,6)+1;
                    end
                else
                    OPEN_COUNT = OPEN_COUNT+1;
                    OPEN(OPEN_COUNT,:) = [TOPUSH(i,1),TOPUSH(i,2),cur_x,cur_y, ...
                                          OPEN(min_h_idx,6)+1+distance(TOPUSH(i,1),TOPUSH(i,2),xTarget,yTarget), ...
                                          OPEN(min_h_idx,6)+1,distance(TOPUSH(i,1),TOPUSH(i,2),xTarget,yTarget)];
                end
            end
        end
        %从将这个数字从OPENLIST中弹出来
        OPEN_COUNT=OPEN_COUNT-1;
        OPEN(min_h_idx,:) = [];        
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    if NoPath 
        path = [];
        return
    end

    path_rev = [];
    path_CNT = 0;
    counter = 1;
    cur_x = xTarget;
    cur_y = yTarget;
    while(1)
        path_CNT = path_CNT+1;
        path_rev(path_CNT,:) = [cur_x,cur_y];
        if(cur_x==xStart&&cur_y==yStart)
            break;
        end
        %找到父节点
        for i = 1:Parent_CNT
            if(Parent(i,1)==cur_x&&cur_y==Parent(i,2))
                cur_x = Parent(i,3);
                cur_y = Parent(i,4);
                break;
            end
        end
    end
    path = zeros(path_CNT,2);
    for i = 1:path_CNT
        path(i,:) = path_rev(path_CNT-i+1,:)-[0.5,0.5];
    end
end
