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

epoch_matrixLB = zeros(size(signL, 1), epoch_size, epoch_number);
epoch_matrixRB = zeros(size(signR, 1), epoch_size, epoch_number);
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
epoch_size=1536;
epoch_matrixLB(1:64,:,:) = LBmiEpo(1:64,:,:);
  epoch_matrixRB(1:64,:,:)=RBmiEpo(1:64,:,:);
%% stampa del segnale con tutte le epoch
t=0:Ts:(length(signL)-1)*Ts;
%figure
%plot(t,signL);title('Time %plot raw signal imagery left');
%%xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t,signR);title('Time %plot raw signal imagery right');
%%xlabel ('Time(s)');ylabel ('Amplitude(V)');

%% first preprocessing
importantChannels=[14,13,12,10,18,48,49,50,46,56];



signLf=reshape(epoch_matrixLB,[size(epoch_matrixLB,3),size(epoch_matrixLB,1),size(epoch_matrixLB,2)]);


signRf=reshape(epoch_matrixRB,[size(epoch_matrixRB,3),size(epoch_matrixRB,1),size(epoch_matrixRB,2)]);



signRfTrain=preprocessingFirstStep(signRf,importantChannels);
signLfTrain=preprocessingFirstStep(signLf,importantChannels);
%figure
%plot(t(1:end),signRfTrain(:,1:end));title('Time %plot filtered signal imagery left');
%%xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t(1:end),signLfTrain(:,1:end)); title('Time %plot filtered signal imagery right');
%%xlabel ('Time(s)');ylabel ('Amplitude(V)');
%%
size(signRfTrain)
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
        
        if  and( mean1+3*std1<meani1,not(ismember(i,esclusi)))
            esclusi=[esclusi i];
            continue
        end
        
        if  and( mean2+3*std2<meani2,not(ismember(i,esclusi)))
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
%xlabel ('Time(s)');ylabel ('Amplitude(V)');


end
%figure
hold on
for i =1:(size(signRfTrain,3))
%plot(signLfTrain(1,:,i)); title('Time %plot signRfTrain signal imagery right');
%xlabel ('Time(s)');ylabel ('Amplitude(V)');
end

%%
size(signRfTrain)

%%
esclusi
size(signRfTrain)
% ExportMatrix= zeros(size(signLfTrain,3),2,size(signLfTrain,1),3,size(signLfTrain,2));
% size(ExportMatrix)
% 
% ExportMatrix(:,1,:,:,:)=preprocessingSecondStep(signRfTrain,false,fc);
% ExportMatrix(:,2,:,:,:)=preprocessingSecondStep(signLfTrain,false,fc);


newMatrix=zeros(size(signLfTrain,3),2,size(signLfTrain,1),3,32);
size(newMatrix)
newMatrix(:,1,:,:,:)=preprocessingSecondStep(signRfTrain,true,fc);
newMatrix(:,2,:,:,:)=preprocessingSecondStep(signLfTrain,true,fc);



% name="Outputs/Labels/"+"AfterPreprocessing"+dataset
% save ([name],"ExportMatrix")

name="Outputs/Labels/"+"AfterPreprocessing"+dataset
save ((name),"newMatrix")


end

