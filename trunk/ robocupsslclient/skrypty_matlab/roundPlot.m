getBall= dlmread( 'blue0round', '; ');
x=getBall(:,1);
y=getBall(:,2);
%xgetBall =1:1:size(getBall)(1);


figure(1);
plot(x, y, 'o')
title('reference');
xlabel('x')
ylabel('y')
