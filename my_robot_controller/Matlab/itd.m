function H=itd(x)
% ITD: Intrinsic Time-Scale Decomposition
% H=itd(x); returns the returns proper rotation components(PRC) and
% residual signal corresponding to the ITD of X
% X should be a 1D signal, H is a 2D matrix, whose rows are PRCs.
% Reference:
% [1] Frei, M. G., & Osorio, I. (2007, February). Intrinsic time-scale decomposition: 
% time-frequency-energy analysis and real-time filtering of non-stationary signals. 
% In Proceedings of the Royal Society of London A: Mathematical, Physical and 
% Engineering Sciences (Vol. 463, No. 2078, pp. 321-342). The Royal Society.
% ------------------------------------------------------------------------
% Written by Linshan Jia (jialinshan123@126.com)
% Xi'an Jiaotong University
% Version 1.0.2
% 2018-11-04

%------------------------------------------------------------------------
N_max=1;
% figure 
% plot(x)
H=[];
xx=x(:)';
E_x=sum(x.^2);
counter=0;
while 1
    counter=counter+1;
    [L1,H1]=itd_baseline_extract(xx);
    if size(L1)==0
        break
    end
    H=[H;H1];
    
    xx=L1;  
    STOP=stop_iter(xx,counter,N_max,E_x);
    if STOP
        H=[H;L1];
        break;
    end
end
% %disp(STOP);
% %disp(counter);
end
%%
% ---------------------------built-in functions---------------------------
%#########################################################################
%#########################################################################
%% function 01: stop the iteration
function STOP=stop_iter(xx,counter,N_max,E_x)
STOP=0;
%------------------------------------
if counter>N_max
    STOP=1;
    return;
end
%------------------------------------
Exx=sum(xx.^2);
if Exx<=0.01*E_x
    STOP=1;

    return;
end
%-------------------------------------
pks1=findpeaks(xx);
pks2=findpeaks(-xx);
pks=union(pks1,pks2);
if length(pks)<=7
    STOP=1;
    return;
end

%EOF
end
%% function 2: extract the baseline
function [L,H]=itd_baseline_extract(x)
% This function is to calculate the baseline L of the signal x
% L and H are row vectors.
% ------------------------------------------------------------------------
% Written by Linshan Jia (jialinshan123@126.com)
% Xi'an Jiaotong University
% Version 1.0.0
% 2018-11-04
x=x(:)';
t=1:length(x);
alpha=0.5;
[val_max, idx_max]=findpeaks(x);
%disp(size(findpeaks(x)));
[val_min, idx_min]=findpeaks(-x);
idx_cb=union(idx_max,idx_min);
val_min=-val_min;
%% boundary conditions
% left side
if min(idx_max)<min(idx_min)
    idx_min=[idx_max(1),idx_min];
    val_min=[val_min(1),val_min];
elseif min(idx_max)>min(idx_min)
    idx_max=[idx_min(1),idx_max];
    val_max=[val_max(1),val_max];    
end
%right side
if max(idx_max)>max(idx_min)
    idx_min=[idx_min,idx_max(end)];
    val_min=[val_min, val_min(end)];
elseif max(idx_max)<max(idx_min)
    idx_max=[idx_max,idx_min(end)];
    val_max=[val_max, val_max(end)];
end
% L=[];
% H=[];
% if and(and(size(val_max)>1 ,size(val_min)>1),and(size(idx_max)>1,size(idx_min)>1))
%% compute the Lk points
Max_line=interp1(idx_max,val_max,t,'linear');
Min_line=interp1(idx_min,val_min,t,'linear');
% figure 
% plot(Max_line)
% hold on 
% plot(Min_line)
% figure
Lk1=alpha*Max_line(idx_min)+val_min*(1-alpha);
Lk2=alpha*Min_line(idx_max)+val_max*(1-alpha);
Lk1=[idx_min(:),Lk1(:)];
Lk2=[idx_max(:),Lk2(:)];
Lk=[Lk1;Lk2];
% plot(Lk)
%  figure
[~,Lk_col_2]=sort(Lk(:,1));
Lk_sorted=Lk(Lk_col_2,:);
% plot(Lk_sorted)
Lk=Lk_sorted(2:end-1,:);
Lk=[[1,Lk(1,2)];Lk;[length(x),Lk(end,2)]];
%% compute the Lt curve
idx_Xk=[1,idx_cb,length(x)];
L=zeros(1,length(x));
for i=1:length(idx_Xk)-1 
    for j=idx_Xk(i):idx_Xk(i+1)
        kij=(Lk(i+1,2)-Lk(i,2))/(x(idx_Xk(i+1))-x(idx_Xk(i))); %compute the slope K
        L(j)=Lk(i,2)+kij*(x(j)-x(idx_Xk(i)));
    end
end
H=x-L;
% figure 
% plot(L)
% figure 
% plot(H)
end
% end