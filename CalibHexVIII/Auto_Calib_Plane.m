clear all;
clc
ofs  = fopen('Plane_Para1.txt','wt');
for i = 0:1:60
    
    clearvars -except i ofs;
    %%原始数据
    pi=3.1415926;
    filename = strcat(num2str(i), '.pcd');
    filename = strcat('./cloud1/cloud',filename);
    Point_Sensor=loadpcd(filename);
    T=[-1,0,0;0,0,1;0,1,0];
    Point_World=zeros(size(Point_Sensor));
    Point_World = T*Point_Sensor;
    Point_World=Point_World';
    
    %%选择数据,剔除掉不是平面的
    j1=1;
    for i1=1:1:size(Point_World,1)
        if (Point_World(i1,1)<=2.6)
            Point_Sel(j1,1)=Point_World(i1,1);
            Point_Sel(j1,2)=Point_World(i1,2);
            Point_Sel(j1,3)=Point_World(i1,3);
            j1=j1+1;
        end
    end
    
    %%选择数据，剔除掉深度等于0的无效点
    j2=1;
    for i2=1:1:size(Point_Sel,1)
        if (Point_Sel(i2,1)~= 'NAN')
            Point_NewSel(j2,1)=Point_Sel(i2,1);
            Point_NewSel(j2,2)=Point_Sel(i2,2);
            Point_NewSel(j2,3)=Point_Sel(i2,3);
            j2=j2+1;
        end
    end
    
    %%X Y Z
    X=Point_NewSel(:,1);
    Y=Point_NewSel(:,2);
    Z=Point_NewSel(:,3);
    
    [Plain,d]=Plain_Fit(X,Y,Z);
    
    while(any(abs(d)>2*Plain(5,1)))
        clear x_new y_new z_new;
        j3=1;
        for i3=1:1:size(d)
            if (abs(d(i3))<=2*Plain(5,1))
                x_new(j3)=X(i3);
                y_new(j3)=Y(i3);
                z_new(j3)=Z(i3);
                j3=j3+1;
            end
        end
        clear X Y Z;
        X= x_new';
        Y= y_new';
        Z= z_new';
        clear d Plain;
        [Plain,d]=Plain_Fit(X,Y,Z);
    end
    
    if Plain(1) > 0
        Plane_Para = -Plain(1:4);
    else
        Plane_Para = Plain(1:4);
    end
    
    fprintf(ofs,'%6.4f %6.4f %6.4f %6.4f\n', Plane_Para);
    
    sprintf('Plane %d finished', i)
    
end