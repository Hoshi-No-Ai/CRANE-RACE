syms x1 y1;

dx=[-552.42 -601.63 -424.52  -83.34  275.20  539.33;
    -552.42 -601.63 -424.55  -81.81  275.20  533.31];
dy=[-263.32  104.75  438.46  611.83  537.41  271.41;
    -263.32  104.75  436.79  611.87  537.41  271.17];
theta=[0.78 1.54 2.00 1.05 0.47  -1.20;
       1.18 1.54 1.59 0.64 0.47  -1.53];
thetaQ=[-90.09 -125.10 -161.14 -198.40 -233.11 -269.56];
x0=zeros(6,1);
y0=zeros(6,1);
 
for i=1:6
    angle=((theta(1,i)+theta(2,i))/2+thetaQ(i))/180*pi;
    e1=(dx(1,i)+dx(2,i))/2+x1*cos(angle)-y1*sin(angle);
    e2=(dy(1,i)+dy(2,i))/2+x1*sin(angle)+y1*cos(angle);
    [x0(i),y0(i)] = solve(e1,e2,x1,y1);
end

x_max=x0(1);
x_min=x0(1);
y_max=y0(1);
y_min=y0(1);
x_id_max=1;
x_id_min=1;
y_id_max=1;
y_id_min=1;
for i=1:6
    if x_max<x0(i)
        x_max=x0(i);
        x_id_max=i;
    end
    if x_min>x0(i)
        x_min=x0(i);
        x_id_min=i;
    end
    if y_max<y0(i)
        y_max=y0(i);
        y_id_max=i;
    end
    if y_min>y0(i)
        y_min=y0(i);
        y_id_min=i;
    end
end

sum_x=0;
sum_y=0;
for i=1:6
    sum_x=sum_x+x0(i);
    sum_y=sum_y+y0(i);
end

average_x=(sum_x-x_min-x_max)/4;
average_y=(sum_y-y_min-y_max)/4;
