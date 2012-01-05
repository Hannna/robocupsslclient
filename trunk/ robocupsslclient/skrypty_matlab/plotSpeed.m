clear all;
close all;
clc;

M = dlmread('blue0',';');
M;

x=1:1:size(M)(1);

vx=M(:,1)
vy=M(:,2)
w=M(:,3)
w_from_filter=M(:,4)


figure(1);
plot(x,vx,'-@');
title('x speed');
xlabel('iteration step')
ylabel('x speed')

      
figure(2);
plot(x,vy);
title('y speed');
xlabel('iteration step')
ylabel('y speed')
   
figure(3);
plot(x,w);
title('w speed');
xlabel('iteration step')
ylabel('w speed')

figure(4);
plot(x,w_from_filter);
title('w_from_filter speed');
xlabel('iteration step')
ylabel('w_from_filter speed')


PID = dlmread('blue0teta',';');
last_teta=PID(:,1);
teta=PID(:,2);
tetad=PID(:,3);
dteta=PID(:,4);
dtetad=PID(:,5);
c=PID(:,6);
delta=PID(:,7);
w_=PID(:,8);
x1 =1:1:size(PID)(1);

%pose = dlmread('blue0xy',';');
%rx=pose(:,1);
%ry=pose(:,2);
%tx=pose(:,3);
%ty=pose(:,4)


figure(5);
plot(x1,last_teta);
title('last_teta');
xlabel('iteration step')
ylabel('last_teta')

figure(6);
plot(x1,teta);
title('teta');
xlabel('iteration step')
ylabel('teta')

figure(7);
plot(x1,tetad);
title('tetad');
xlabel('iteration step')
ylabel('tetad')

figure(8);
plot(x1,dteta);
title('dteta');
xlabel('iteration step')
ylabel('dteta')

figure(9);
plot(x1,dtetad);
title('dtetad');
xlabel('iteration step')
ylabel('dtetad')

figure(10);
plot(x1,c);
title('c');
xlabel('iteration step')
ylabel('c')

figure(11);
plot(x1,delta);
title('delta tetad');
xlabel('iteration step')
ylabel('delta')


figure(12);
plot(x, tetad( 1:1:size(x,2) ), x,teta(1:1:size(x,2) ) ,x,w_from_filter, x, w_(1:1:size(x,2) )  );
title('delta tetad');
xlabel('iteration step')
ylabel('delta')
legend ("tetad","teta", "w","w from PID");


%hold on
%figure(12);
%plot(ry,rx,ty,tx,'--rs','LineWidth',2,
  %              'MarkerEdgeColor','k',
    %            'MarkerFaceColor','g',
      %         'MarkerSize',2)

%plot(tx,ty);
%title('polozenie robota');
%xlabel('x')
%ylabel('y')
%hold off

%figure(11);
%plot(tx,ty);
%title('polozenie celu');
%xlabel('x')
%ylabel('y')

%v = dlmread('blue0v',';');
%gx=v(:,1);
%gy=v(:,2);
%rx=v(:,3);
%ry=v(:,4);
%xv =1:1:size(v)(1);

%figure(13);
%plot(gx,gy);
%title('goal pose');
%xlabel('x')
%ylabel('y')

%figure(14);
%plot(rx,ry);
%title('robot pose');
%xlabel('x')
%ylabel('y')

%getBall= dlmread( 'blue0getBall' , '; ');
%reference=getBall(:,1);
%angle=getBall(:,2);
%xgetBall =1:1:size(getBall)(1);


%figure(15);
%plot(xgetBall,reference);
%title('reference');
%xlabel('x')
%ylabel('y')

%figure(16);
%plot(xgetBall,angle);
%title('angle to ball');
%xlabel('x')
%ylabel('y')
