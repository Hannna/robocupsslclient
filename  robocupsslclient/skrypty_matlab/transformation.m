
 function  [x1,y1,angle1] =  transformation( x0, y0 , angle0, fi , d) 
% x0, y0 wspolrzedne w globalnym ukladzie odniesienia angle0 kat do osi oy w globalnym ukladzie odniesienia
%fi kat o jaki trzeba obrocic uklad globalby aby osie pokryly sie z loklanym
	
%this->fi=fi;
%a11=cos(fi);
%a12=-sin(fi);
%a21=sin(fi);
%a22=cos(fi);
%

rotation=[cos(fi),-sin(fi); sin(fi),cos(fi) ]
inv_rotation = inv(rotation)
dd= [x0,y0]'-d
a = inv_rotation*dd

angle1 = 0;
x1=a(1);
y1=a(2);