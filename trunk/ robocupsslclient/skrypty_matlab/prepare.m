clc
clear all;


%mydata = rand(400,2); % Make some data
%plot(mydata(:,1),mydata(:,2),'.') % Plot it
%tickL = get(gca,'yticklabel') % Get the tick labels (initially as a string)
% Convert them to numbers, multiply by 100, convert back to string, add a '%' symbol.
%newTickL = strcat(num2str(str2num(tickL)*100),'%')
%newTickL = strcat(num2str(1:10:12)*100),'%')
%set(gca, 'ytickLabel', newTickL) % Update the labels 

%h=gca;
%labels=get(h,'yticklabel'); % get the y axis labels
%labels_modif=[num2str(100*str2num(labels)) ones(length(labels),1)*'%']
%labels_modif
%set(h,'yticklabel',labels_modif); 



kwant = 0.1;%input('Podaj kwant uzyty do eksperymentow: ');
l_eksperymentow = 10;
l_powtorzen = 17;

sciezka = '../Debug/cvm_prosty/L15/przod';
sciezka = '../Debug/wyniki_rzech';

i=0;
pg=[];
pw=[];
for pgoal = 0:kwant:1
    for pway = 0:kwant:1
            if abs( ( pgoal+pway ) ) <=1.0 & abs( ( pgoal+pway ) ) > 0.0
                i = i+1;
                pg(i) = pgoal;
                pw(i) = pway;
                fprintf('Zestaw wag nr %d:\t%f %f %f\n',i,pgoal ,pway );                
            end
    end
end

A=[pg;pw]
save -ascii myfile.txt A


ilosc_wag = i;

sukcesy_nrWagi = zeros(ilosc_wag,l_powtorzen);
sukcesy_nrPlanszy = zeros(ilosc_wag,l_eksperymentow);
czas_dojazdu_nrWagi = zeros(ilosc_wag,l_powtorzen);
total_rrt_time_nrWagi = zeros(ilosc_wag,l_powtorzen);
rrt_iterations_nrWagi = zeros(ilosc_wag,l_powtorzen);
max_path_size_nrWagi = zeros(ilosc_wag,l_powtorzen);
min_path_size_nrWagi = zeros(ilosc_wag,l_powtorzen);
max_tree_size_nrWagi = zeros(ilosc_wag,l_powtorzen);
min_tree_size_nrWagi = zeros(ilosc_wag,l_powtorzen);
path_found_nr_wagi = zeros(ilosc_wag,l_powtorzen);

for series_nr=1:1:l_powtorzen
    
     nazwa = [ './dynamic/tmp' num2str(series_nr) '.txt'];
   
    M = dlmread(nazwa,'\t');
    pgoal = M(:,1);
    pway = M(:,2);
    nodeAmount= M(:,3);
    srodowisko= M(:,4);
    czasRozp= M(:,5);
    czasZak= M(:,6);
    ukonczono= M(:,7);
    czasSym= M(:,8);
    total_rrt_time= M(:,9);
    rrt_iterations= M(:,10);
    max_path_size= M(:,11);
    min_path_size= M(:,12);
    max_tree_size= M(:,13);
    min_tree_size= M(:,14);
    
    nr_eksperymentu = 1;
    nr_zestawu=1;
    for j = 2: size(pgoal,1)
        
        sukcesy_nrWagi(nr_zestawu,series_nr) = sukcesy_nrWagi(nr_zestawu,series_nr) + ukonczono(j);
        sukcesy_nrPlanszy(nr_zestawu,nr_eksperymentu) = sukcesy_nrPlanszy(nr_zestawu,nr_eksperymentu) + ukonczono(j);
        
        czas_dojazdu_nrWagi(nr_zestawu,series_nr) = czas_dojazdu_nrWagi(nr_zestawu,series_nr) + czasSym(j);
        
        total_rrt_time_nrWagi(nr_zestawu,series_nr)  =  total_rrt_time_nrWagi(nr_zestawu,series_nr) + total_rrt_time(j);
        rrt_iterations_nrWagi(nr_zestawu,series_nr)  =  rrt_iterations_nrWagi(nr_zestawu,series_nr) + rrt_iterations(j);
        
        if max_path_size(j) >0
        	max_path_size_nrWagi(nr_zestawu,series_nr) = max_path_size_nrWagi(nr_zestawu,series_nr) +max_path_size(j) ;
        	path_found_nr_wagi(nr_zestawu, series_nr)=path_found_nr_wagi(nr_zestawu,series_nr)+1;
        end
        
        if max_path_size(j) >0
        	max_path_size_nrWagi(nr_zestawu,series_nr) =max_path_size_nrWagi(nr_zestawu,series_nr) + max_path_size(j) ;
        end
        
        if max_tree_size(j) >0
        	max_tree_size_nrWagi(nr_zestawu,series_nr) = max_tree_size_nrWagi(nr_zestawu,series_nr)  + max_tree_size(j);
        end
        
        nr_eksperymentu = nr_eksperymentu + 1;
        
        if nr_eksperymentu > l_eksperymentow
           nr_eksperymentu = 1;
           
           sukcesy_nrWagi(nr_zestawu,series_nr) = (sukcesy_nrWagi(nr_zestawu,series_nr)/l_eksperymentow)*100;
           czas_dojazdu_nrWagi(nr_zestawu,series_nr) = czas_dojazdu_nrWagi(nr_zestawu,series_nr)/l_eksperymentow;
           
            total_rrt_time_nrWagi(nr_zestawu,series_nr)=total_rrt_time_nrWagi(nr_zestawu,series_nr)/rrt_iterations_nrWagi(nr_zestawu,series_nr);
            
            nr_zestawu = nr_zestawu+1;
        end
    end
    
end

x_lab = 1:1:ilosc_wag;
sukcesy_nrWagi
figure(1);
y = mean(sukcesy_nrWagi,2);
e = std(sukcesy_nrWagi,1,2);
errorbar(y,e);
axis([0 ilosc_wag+1 0 120])
xlabel('numer wagi')
ylabel('procent sukcesow')
title('liczba eksperymentow zakonczonych dojazde do celu')
print -deps foo1.eps

%aa=0:2:12;
%aa=aa*10;
%newTickL=num2str(aa(1,1:1:7) );
%newTickL=strcat(newTickL, '%')
%set(gca, 'ytickLabel', {'0%';'20%';'40%';'60%';'80%';'100%';'120%'}) % Update the labels 
%axis([0 ilosc_wag+1 0 12])
%xlabel('numer wagi')
%ylabel('procent sukcesow')


figure(2);
yy = mean( czas_dojazdu_nrWagi,2);
ee = std( czas_dojazdu_nrWagi,1,2);
errorbar(yy,ee);
axis([0 ilosc_wag+1 0 12])
xlabel('numer wagi')
ylabel('czas [s]')
title('czas dojazdu do celu ')
print -deps foo2.eps

figure(3);
yyy = mean( total_rrt_time_nrWagi,2);
eee = std( total_rrt_time_nrWagi,1,2);
errorbar(yyy,eee);
xlim([0 ilosc_wag+1 ])
ylim([0 400 ])
%axis([0 ilosc_wag+1 0 12])
xlabel('numer wagi')
ylabel('czas [ms]')
title('sredni czas obliczen algorytmu rrt dla danego zestawy wag')
print -deps foo3.eps

figure(4);
#bar( x_lab,path_found_nr_wagi );
yyy_ = mean( path_found_nr_wagi,2);
eee_ = std( path_found_nr_wagi,1,2);
errorbar(yyy_,eee_);
axis([0 ilosc_wag+1 0 12])
%xlim([0 ilosc_wag+1 ])
xlabel('nr zestawu wag');
ylabel('liczba sciezek do celu');
title('liczba znalezionych sciezek do celu dla danego zestwu wag')
print -deps foo4.eps

figure(5);
yyyy = mean( max_path_size_nrWagi,2);
eeee = std( max_path_size_nrWagi,1,2);
errorbar(yyyy,eeee);
xlim([0 ilosc_wag+1 ])
ylim([0 700 ])
%axis([0 ilosc_wag+1 0 12])
xlabel('numer wagi')
ylabel('rozmiar sciezki')
title('maksymalny rozmiar sciezki dla danego zestwu wag')
print -deps foo5.eps


figure(6);
yyyyy = mean(max_tree_size_nrWagi,2);
eeeee = std( max_tree_size_nrWagi,1,2);
errorbar(yyyyy,eeeee);
xlim([0 ilosc_wag+1 ])
%axis([0 ilosc_wag+1 0 12])
xlabel('numer wagi')
ylabel('rozmiar sciezki')
title('maksymalny rozmiar drzewa dla danego zestwu wag')
print -deps foo6.eps




%labels_modif=[num2str(100*str2num(labels)) ones(length(labels),1)*'%']
%set(h,'yticklabel',labels_modif); 


%bar(x_lab,mean(sukcesy_nrWagi,2));
%xlabel('nr zestawu wag');
%ylabel('srednia liczba sukcesow');

%figure(2);
%bar(x_lab,std(sukcesy_nrWagi,1,2));
%xlabel('nr zestawu wag');
%ylabel('odchylenie std');
pause
close all

