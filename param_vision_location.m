syms x1 y1;

dx=[-414.43 -497.19 -380.70  -124.81  170.75  408.83;
    -414.43 -497.19 -380.70  -124.81  170.75  408.83];
dy=[-266.43  22.17  307.05  479.68  468.82  261.68;
    -266.43  22.17  307.05  479.68  468.82  261.68];
theta=[-1.66 -0.5 -1.16 -1.24 -1.84  -1.15;
       -1.66 -0.5 -1.16 -1.24 -1.84  -1.15];
thetaQ=[-90.19 -125.41 -162.06 -197.69 -231.88 -269.86];
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

thetaa=-2.24;
thetaQQ=-89.99;
dxx=-422.55;
dyy=-272.48;
x00=zeros(1,1);
y00=zeros(1,1);
angle=(thetaa+thetaQQ)/180*pi;
e1=dxx+x1*cos(angle)-y1*sin(angle);
e2=dyy+x1*sin(angle)+y1*cos(angle);
[x00(1,1),y00(1,1)] = solve(e1,e2,x1,y1);
