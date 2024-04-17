%caricamento dati
allDataset=52

for count=1:allDataset
if count <10
    dataset="0"+count
else
    dataset=""+count
end
signL=[]
signR=[]
epoch_size = 3584;


current="s"+dataset+".mat"
load (current)
eeg.n_imagery_trials
signL=[signL eeg.imagery_left(1:64,1:100*epoch_size)];
signR=[signR eeg.imagery_right(1:64,1:100*epoch_size)];

%%

% for i=1:64
% signL(i,:)=signL(i,:)-mean(signL(i,:));
% signR(i,:)=signR(i,:)-mean(signR(i,:));
% end

fc=eeg.srate;
Ts=1/fc;
trials=eeg.n_imagery_trials;
eventiIm=eeg.imagery_event;
%% 
%Scompongo in epoche
epoch_size = 3584;
epoch_number = size(signL, 2)/epoch_size;

epoch_offset = 1;
epoch_time = (0:epoch_size-1) / fc;
daRimuovere=[]%1, 21, 41, 61, 81 , 98 ];%Rimuovo le epoche "strane" dello scorso task

epoch_matrixLB = zeros(size(signL, 1), epoch_size, epoch_number-length(daRimuovere));
epoch_matrixRB = zeros(size(signR, 1), epoch_size, epoch_number-length(daRimuovere));
index=0;
%riempiamo le matrici
for i = 1:(epoch_number)
  index=index+1;
  indexes = epoch_offset+((i-1)*epoch_size):1:epoch_offset+(i*epoch_size)-1;
   epoch_matrixLB(1:64,:,index) = signL(1:64, indexes);
  epoch_matrixRB(1:64,:,index) = signR(1:64, indexes);
end
 LBmiEpo = epoch_matrixLB(1:64,1025:2560,:);
RBmiEpo = epoch_matrixRB(1:64,1025:2560,:);
epoch_matrixRB=[];
epoch_matrixLB=[];
epoch_matrixLB(1:64,:,:) = LBmiEpo(1:64,:,:);
  epoch_matrixRB(1:64,:,:)=RBmiEpo(1:64,:,:);
%% stampa del segnale con tutte le epoch
t=0:Ts:(length(signL)-1)*Ts;
%figure
%plot(t,signL);title('Time %plot raw signal imagery left');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t,signR);title('Time %plot raw signal imagery right');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
%% 
epoch_size=size(epoch_matrixLB, 2);
% prendiamo le serie di epoche divise
signalL=zeros(size(signL, 1), epoch_size*(epoch_number-length(daRimuovere)));
signalR=zeros(size(signR, 1), epoch_size*(epoch_number-length(daRimuovere)));

for i = 1:(epoch_number-length(daRimuovere))
     indexes = epoch_offset+((i-1)*epoch_size):1:epoch_offset+(i*epoch_size)-1;

    signalL(1:64,indexes)=epoch_matrixLB(1:64,:,i);
    signalR(1:64,indexes)=epoch_matrixRB(1:64,:,i);

end
signL=signalL;
signR=signalR;
%% segnale senza epoche anomale
t=0:Ts:(length(signL)-1)*Ts;
%figure
%plot(t,signL);title('Time %plot raw signal imagery left');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t,signR);title('Time %plot raw signal imagery right');
xlabel ('Time(s)');ylabel ('Amplitude(V)');




%% filtraggio


num=epoch_size*(epoch_number-length(daRimuovere));

%filtraggio 8-30 Hz
signLf=zeros(64,num);
for i=1:1:64;signLf(i,:)=filter(filtroPB,signL(i,:));end

signRf=zeros(64,num);
for i=1:1:64;signRf(i,:)=filter(filtroPB,signR(i,:));end
for i=1:64
signLf(i,:)=signLf(i,:)-mean(signLf(i,:));
signRf(i,:)=signRf(i,:)-mean(signRf(i,:));
end
%% common average

t=0:Ts:(length(signLf)-1)*Ts;
%figure
%plot(t(1:end),signRf(:,1:end));title('Time %plot filtered signal imagery left');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t(1:end),signRf(:,1:end)); title('Time %plot filtered signal imagery right');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
% Split data
% %disp(num-limitTrain)

importantChannels=[14,13,12,10,18,48,49,50,46,56]


num=epoch_size*(epoch_number-length(daRimuovere));
for i =1:epoch_size
    for e= importantChannels
    signRf(e,i)=signRf(e,i)-mean(signRf(:,i));
    signLf(e,i)=signLf(e,i)-mean(signLf(:,i));

    end
end

signRfTrain=signRf(importantChannels,:);
signLfTrain=signLf(importantChannels,:);
%figure
%plot(t(1:end),signRfTrain(:,1:end));title('Time %plot filtered signal imagery left');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t(1:end),signLfTrain(:,1:end)); title('Time %plot filtered signal imagery right');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
%%
signRfTrain=reshape(signRfTrain,[length(importantChannels),epoch_size,index]);
signLfTrain=reshape(signLfTrain,[length(importantChannels),epoch_size,index]);
%% outlier test 1.5 delta (strict control)
% size(signLfTrain(1,:))
esclusi=[];
mean1=0;
std1=0;
std2=0;
mean2=0;

for e = 1:(size(signLfTrain,1))

    S_sum1=0;
    S_sum2=0;
    for i = 1:(size(signLfTrain,3))
        S_sum1=S_sum1+std(abs(signLfTrain(e,:,i)));
        S_sum2=S_sum2+std(abs(signRfTrain(e,:,i)));
    end
    mean1=mean(abs(signLfTrain(e,:)));
    mean2=mean(abs(signRfTrain(e,:)));
    std1=S_sum1/size(signLfTrain,3);
    std2=S_sum2/size(signRfTrain,3);

    for i = 1:(size(signLfTrain,3))

        meani1=mean(abs(signLfTrain(e,:,i)));
        meani2=mean(abs(signRfTrain(e,:,i)));
        
        if  and( mean1+1.5*std1<meani1,not(ismember(i,esclusi)))
            esclusi=[esclusi i];
            continue
        end
        
        if  and( mean2+1.5*std2<meani2,not(ismember(i,esclusi)))
            esclusi=[esclusi i];
            continue
        end
    
    
    
    end
end
esclusi

%% 


%% rimozione di epoche anomale
newMatrix=zeros(size(signRfTrain,1),size(signLfTrain,2),size(signLfTrain,3)-length(esclusi));
indice=1;
for i =1:(size(signLfTrain,3))
    if ismember(i,esclusi)
        continue
    end
    newMatrix(:,:,indice)=signLfTrain(:,:,i);
    indice=indice +1;
end
signLfTrain=newMatrix;

newMatrix=zeros(size(signRfTrain,1),size(signLfTrain,2),size(signLfTrain,3)-length(esclusi));
indice=1;
for i =1:(size(signRfTrain,3))
    if ismember(i,esclusi)
        continue
    end
    newMatrix(:,:,indice)=signRfTrain(:,:,i);
    indice=indice +1;
end

signRfTrain=newMatrix;
%figure
hold on
for i =1:(size(signRfTrain,3))
%plot(signRfTrain(1,:,i));title('Time %plot signRfTrain signal imagery left');
xlabel ('Time(s)');ylabel ('Amplitude(V)');


end
%figure
hold on
for i =1:(size(signRfTrain,3))
%plot(signLfTrain(1,:,i)); title('Time %plot signRfTrain signal imagery right');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
end

%%
size(signRfTrain)

%%
signRfTrain2=[];
signRfTest2=[];
signLfTrain2=[];
signLfTest2=[];
size(signRfTrain)
ExportMatrix= zeros(size(signLfTrain,3),2,size(signLfTrain,1),3,size(signLfTrain,2));
size(ExportMatrix)
%%
discardIndex=[];
%figure
hold on
for i = 1:(size(ExportMatrix,1))
   for e=1:length(importantChannels)
    try
        try1=itd(signRfTrain(e,:,i));
    
    catch
        if not(ismember(i, discardIndex))
            %plot(signRfTrain(e,:,i))
            
            discardIndex=[discardIndex ,i];
            break
        end
    end
    ExportMatrix(i,1,e,:,:)=try1(1:3,:);
   end
    
end
discardIndex
%%
% %plot(       reshape(ExportMatrix(1,1,1,1,:),1,[]) )
%%
size(ExportMatrix)

%%
for i = 1:(size(ExportMatrix,1))
   for e=1:length(importantChannels)
    
    try1=itd(signLfTrain(e,:,i));
    %disp("size"+size(try1,1));
    
    if and(size(try1,1)< 3,not(ismember(i, discardIndex)))

        discardIndex=[discardIndex ,i];
        break
    end
    
    ExportMatrix(i,2,e,:,:)=try1(1:3,:);
   end
    
end
discardIndex
min
%%
% %figure
% temp=reshape(ExportMatrix(2,1,1,1,:),1,[]);
% %plot(temp)
% hold on 
% for i=2:3
%     temp= reshape(ExportMatrix(2,1,1,i,:),1,[]);
%     %plot (temp)
% end
% 
% %plot( temp );
% %figure
% hold on
% %plot(signRfTrain(1,:,2))
%%
newMatrix=zeros(size(ExportMatrix,1)-length(discardIndex),size(ExportMatrix,2),size(ExportMatrix,3),size(ExportMatrix,4),size(ExportMatrix,5));
indice=1;
for i =1:(size(ExportMatrix,1))
    if ismember(i,discardIndex)
        continue
    end
    newMatrix(indice,:,:,:,:)=ExportMatrix(i,:,:,:,:);
    indice=indice +1;
end
ExportMatrix=newMatrix;
%%
size(ExportMatrix)
%figure
hold on
for i =1:(size(ExportMatrix,1))
%plot(reshape(ExportMatrix(i,1,1,1,:),1,[]));title('Time %plot ExportMatrix signal imagery left');
xlabel ('Time(s)');ylabel ('Amplitude(V)');


end
%figure
hold on
for i =1:(size(ExportMatrix,1))
%plot(reshape(ExportMatrix(i,2,1,1,:),1,[])) ;title('Time %plot ExportMatrix signal imagery right');
xlabel ('Time(s)');ylabel ('Amplitude(V)');
end
%% saving
%save exportToPy ExportMatrix

%%
newMatrix=zeros(size(ExportMatrix,1),size(ExportMatrix,2),size(ExportMatrix,3),size(ExportMatrix,4),32);
size(ExportMatrix)
size(newMatrix)
 for a =1:(size(ExportMatrix,1))
     for e =1:size(ExportMatrix,2)
        for i= 1:size(ExportMatrix,3)
            for o=1:size(ExportMatrix,4)
                temp=reshape(ExportMatrix(a,e,i,o,:),1,size(ExportMatrix,5));
                %%disp energia
                newMatrix(a,e,i,o,1)=max(xcorr(temp,temp));
                %%disp potenza
                newMatrix(a,e,i,o,2)=bandpower(temp);
                %entropia instantaneous è una variabile che permette di
                %inibire l'operazione di media sulle varie frequenze
                newMatrix(a,e,i,o,3:end)=pentropy(temp,fc,'FrequencyLimits',[8 30],'Instantaneous', true);
                
                %pkurtosis(try1(e,:)

            end
        end
     end
 end
 %%
 %figure
 %disp energia
%disp (newMatrix(1,1,2,1,1))
%disp potenza
%disp (newMatrix(1,1,2,1,2))
%plot(reshape(newMatrix(1,1,2,1,3:end),1,30));

hold on
%disp energia
%disp (newMatrix(1,1,3,1,1))
%disp potenza
%disp (newMatrix(1,1,3,1,2))
%plot(reshape(newMatrix(1,1,3,1,3:end),1,30));
size(newMatrix)
%% saving initial data without discarded epochs
Matrix=zeros(2,size(epoch_matrixRB,1),size(epoch_matrixRB,2),(size(epoch_matrixRB,3)-length(esclusi)-length(discardIndex)));
size(Matrix)
size(epoch_matrixRB);
index1=1
for i= 1:size(epoch_matrixRB,3)
if not(ismember(i, discardIndex)||ismember(i, esclusi)) 
    Matrix(1,:,:,index1)=epoch_matrixRB(:,:,i);
    Matrix(2,:,:,index1)=epoch_matrixLB(:,:,i);
    index1=index1+1;

end
size(Matrix(1,1,:,1));

%plot(reshape(Matrix(1,1,:,1),1,[]))
end
%% saving 

%%
%name="Outputs/Observations/"+"PrePreprocessing"+dataset
%save ([name],"Matrix")
name="Outputs/Labels/"+"AfterPreprocessing"+dataset
save ([name],"newMatrix")


end

