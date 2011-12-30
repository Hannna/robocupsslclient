clc
clear all;

[alfa1,alfa2,alfa3,srodowisko,czasRozp,czasZak,ukonczono,czasSym]=textread('../Debug/wynik1_kamil.txt','%f %f %f %25s %25s %25s %f %f','delimiter','\t','headerlines',1);




kwant = input('Podaj kwant uzyty do eksperymentow: ');


%for i=1:size(alfa1,1)
 %   fprintf('Zestaw wag: %f %f %f,  stosunki b/a %f c/a %f \n',alfa1(i),alfa2(i),alfa3(i),alfa2(i) / alfa1(i),alfa3(i) / alfa1(i));

%end

i = 0;
for a = kwant:kwant:1
    for b = 0:kwant:1
        for c = 0:kwant:1
            if abs((a+b+c)-1) <kwant/10
                i = i + 1;
                %fprintf('%f %f %f\n',a ,b ,c);
                
                ba(i) = b/a;
                ca(i) = c/a;
                
            end
        end    
    end
end

i;

ba = unique(ba);
ca = unique(ca);


z = zeros(size(ba,2));
time = zeros(size(ba,2));

for i=1:size(alfa1,1)
    s_ba = alfa2(i) / alfa1(i);
    s_ca = alfa3(i) / alfa1(i);
    index_ba = find(ba > s_ba - kwant/10.0 & ba < s_ba + kwant/10.0);
    index_ca = find(ca > s_ca - kwant/10.0 & ca < s_ca + kwant/10.0);
    z(index_ba,index_ca) = z(index_ba,index_ca) + ukonczono(i);
    %
    time(index_ba,index_ca) = time(index_ba,index_ca) + ukonczono(i)*czasSym(i);
end


for i=1:size(time,1)
    for j = 1: size(time,2)
        if z(i,j)>0 
            time(i,j) = time(i,j) / z(i,j);
        end
    end
end

figure(1);
mesh(ba,ca,z);
title('Liczba sukcesow');
xlabel('b/a'); ylabel('c/a'); zlabel('Liczba sukcesow');


figure(2);
mesh(ba,ca,time);
xlabel('b/a'); ylabel('c/a'); zlabel('Czas eksperymentu [s]');



pause;
close all;


liczba_sukcesow = zeros(1,size(alfa1,1));
czasy = zeros(1,size(alfa1,1));

a0 = alfa1(1);
b0 = alfa2(1);
c0 = alfa3(1);

j=1;
k=1;
tmp = zeros(1,20);

fprintf('Zestaw wag nr %d: %f %f %f\n',j,a0,b0,c0);

for i=1:size(alfa1,1)
    if (a0 ~= alfa1(i) || b0 ~= alfa2(i) || c0 ~= alfa3(i))
        if (size(find(tmp>0),2) > 0)
            czasy(j) = mean(tmp(find(tmp>0)));
        end
        
        j = j+1;
        a0 = alfa1(i);
        b0 = alfa2(i);
        c0 = alfa3(i);
        fprintf('Zestaw wag nr %d: %f %f %f\n',j,a0,b0,c0);
        
        k=1;
        tmp = zeros(1,20);
    end
    
    tmp(k) = ukonczono(i) * czasSym(i);
    k = k+1;
    
    liczba_sukcesow(j) = liczba_sukcesow(j) + ukonczono(i);
end 

x_lab = find(liczba_sukcesow>0);
l_sukc_efektywna = liczba_sukcesow(find(liczba_sukcesow>0));

figure(1);
bar(x_lab,l_sukc_efektywna);
xlabel('nr zestawu wag');
ylabel('liczba sukcesow');

x_lab = find(czasy>0);
figure(2);
bar(x_lab,czasy(czasy>0));
xlabel('nr zestawu wag');
ylabel('sredni czas (dla sukcesow)');

pause;
close all;


