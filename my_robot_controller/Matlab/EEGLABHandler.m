EEG.etc.eeglabvers = '2024.0'; % this tracks which version of EEGLAB is being used, you may ignore it
    eeglab;
    right=[];
    left=[];
    steady=[];
for index=1:3
    path=strcat(['/home/marco/catkin_ws/src/my_robot_controller/scripts/modelEmotiv/Gregorio/greg'],int2str(index),'.gdf');
    EEG = pop_biosig(path);
    EEG = pop_reref( EEG, []);
    EEG = pop_resample( EEG, 512);
    edftype=[EEG.event.edftype];
    latency=[EEG.event.latency];
    duration=[EEG.event.duration];
    size(EEG.data,2)
    disp("")
    length(edftype);
    for i = 1:length(edftype)
        duration(i)
        latency(i)+1535
        if(size(EEG.data,2)<latency(i)+1535)
            continue
        end
        
        if(edftype(i)==773)
            temp=EEG.data(:,latency(i):latency(i)+1535);
            right=cat(1, right, reshape(temp, 1, EEG.nbchan, EEG.srate*3));
        elseif edftype(i) == 786
            temp=EEG.data(:,latency(i):latency(i)+1535);
            steady=cat(1, steady, reshape(temp, 1, EEG.nbchan, EEG.srate*3));
        elseif edftype(i) == 771
            temp=EEG.data(:,latency(i):latency(i)+1535);
            left=cat(1, left, reshape(temp, 1, EEG.nbchan, EEG.srate*3));
        end
    end

end
finalMatrix=zeros([3,size(right)]);
size(finalMatrix);
size(finalMatrix(1,:,:,:));
finalMatrix(1,:,:,:)=right;
finalMatrix(2,:,:,:)=left(length(right));
finalMatrix(3,:,:,:)=steady(length(right));
dim=size(finalMatrix)
epoch_matrixRB=reshape(finalMatrix(1,:,:,:),[dim(3),dim(4),dim(2)]);
epoch_matrixLB=reshape(finalMatrix(2,:,:,:),[dim(3),dim(4),dim(2)]);
epoch_RRest=reshape(finalMatrix(3,:,:,:),[dim(3),dim(4),dim(2)]);
%% first preprocessing
importantChannels=1:14;
% r = int8(1+(10-1).* rand(1,1));
% 
% noisedChannels=importantChannels(r);
% %noisedChannels=[10,18,56,46];
% %epoch_RRest(noisedChannels,:,:)=epoch_RRest(noisedChannels,:,:)+noise(1,1:1536,:);
% 
% %epoch_LRest(noisedChannels,:,:)=epoch_LRest(noisedChannels,:,:)+noise(1,1:1536,:);
% 
% epoch_matrixRB(noisedChannels,:,:)=epoch_matrixRB(noisedChannels,:,:)+noise(1,1:1536,:);
% % r = int8(1+(10-1).* rand(4,1));
% % noisedChannels=importantChannels(r);
% epoch_matrixLB(noisedChannels,:,:)=epoch_matrixLB(noisedChannels,:,:)+noise(1,1:1536,:);

% signLf=reshape(epoch_matrixLB,[size(epoch_matrixLB,3),size(epoch_matrixLB,1),size(epoch_matrixLB,2)]);
% 
% 
% signRf=reshape(epoch_matrixRB,[size(epoch_matrixRB,3),size(epoch_matrixRB,1),size(epoch_matrixRB,2)]);
% signLfRest=reshape(epoch_LRest,[size(epoch_LRest,3),size(epoch_LRest,1),size(epoch_LRest,2)]);
% 
% 
% signRfRest=reshape(epoch_RRest,[size(epoch_RRest,3),size(epoch_RRest,1),size(epoch_RRest,2)]);

size(epoch_RRest)
signRfRestTrain=preprocessingFirstStep(epoch_RRest,importantChannels);

signLfTrain=preprocessingFirstStep(epoch_matrixLB,importantChannels);

signRfTrain=preprocessingFirstStep(epoch_matrixRB,importantChannels);

%figure
%plot(t(1:end),signRfTrain(:,1:end));title('Time %plot filtered signal imagery left');
%%xlabel ('Time(s)');ylabel ('Amplitude(V)');
%figure
%plot(t(1:end),signLfTrain(:,1:end)); title('Time %plot filtered signal imagery right');
%%xlabel ('Time(s)');ylabel ('Amplitude(V)');
%%
% size(signRfTrain)
% signRfRestTrain=reshape(signRfRestTrain,[length(importantChannels),epoch_size,index]);
% signLfRestTrain=reshape(signLfRestTrain,[length(importantChannels),epoch_size,index]);
% signRfTrain=reshape(signRfTrain,[length(importantChannels),epoch_size,index]);
% signLfTrain=reshape(signLfTrain,[length(importantChannels),epoch_size,index]);
%% outlier test 3 delta 
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
newMatrix=zeros(size(signLfTrain,1),size(signLfTrain,2),size(signLfTrain,3)-length(esclusi));
newMatrixRest=zeros(size(signRfRestTrain,1),size(signRfRestTrain,2),size(signRfRestTrain,3)-length(esclusi));
indice=1;
for i =1:(size(signLfTrain,3))
    if ismember(i,esclusi)
        continue
    end
    newMatrix(:,:,indice)=signLfTrain(:,:,i);
    indice=indice +1;
end
signLfTrain=newMatrix;
newMatrix=zeros(size(signRfTrain,1),size(signRfTrain,2),size(signRfTrain,3)-length(esclusi));
newMatrixRest=zeros(size(signRfRestTrain,1),size(signRfRestTrain,2),size(signRfRestTrain,3)-length(esclusi));
indice=1;
for i =1:(size(signRfTrain,3))
    if ismember(i,esclusi)
        continue
    end
    newMatrix(:,:,indice)=signRfTrain(:,:,i);
    newMatrixRest(:,:,indice)=signRfRestTrain(:,:,i);
    indice=indice +1;
end

signRfTrain=newMatrix;
signRfRestTrain=newMatrixRest;

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
%% Extraction of signals performing itd
% options=[true,false];
% ExportMatrix= zeros(size(signLfTrain,3),4,size(signLfTrain,1),3,size(signLfTrain,2));
% size(ExportMatrix)
% 
% ExportMatrix(:,1,:,:,:)=preprocessingSecondStep(signRfTrain,options,fc);
% ExportMatrix(:,2,:,:,:)=preprocessingSecondStep(signLfTrain,options,fc);
% ExportMatrix(:,3,:,:,:)=preprocessingSecondStep(signRfRestTrain,options,fc);
% 
% name="Outputs/Labels/"+"AfterPreprocessing"+dataset
% save ((name),"ExportMatrix")
%% Extraction of signals without performing itd
% ExportMatrix= zeros(size(signLfTrain,3),3,size(signLfTrain,1),size(signLfTrain,2));
% size(ExportMatrix)
% 
% ExportMatrix(:,1,:,:)=permute(signRfTrain,[3 1 2]);
% ExportMatrix(:,2,:,:)=permute(signLfTrain,[3 1 2]);
% ExportMatrix(:,3,:,:)=permute(signRfRestTrain,[3 1 2]);
% 
% 
% name="datasetEEGNET";
% save ([name],"ExportMatrix")

%% Extraction of metrics on signal performing itd
% options=[true,true];
% newMatrix=zeros(size(signLfTrain,3),3,size(signLfTrain,1),3,32);
% size(newMatrix)
% newMatrix(:,1,:,:,:)=preprocessingSecondStep(signRfTrain,options,fc);
% newMatrix(:,2,:,:,:)=preprocessingSecondStep(signLfTrain,options,fc);
% newMatrix(:,3,:,:,:)=preprocessingSecondStep(signRfRestTrain,options,fc);
% name="Outputs/Labels/"+"AfterPreprocessingMetrics"+dataset
% save ((name),"newMatrix")
% Extraction of metrics on signal without performing itd
options=[false,true];
newMatrix=zeros(size(signLfTrain,3),3,size(signLfTrain,1),32);
size(newMatrix)
fc=256;

newMatrix(:,1,:,:)=preprocessingSecondStep(signRfTrain,options,fc);
newMatrix(:,2,:,:)=preprocessingSecondStep(signLfTrain,options,fc);
newMatrix(:,3,:,:)=preprocessingSecondStep(signRfRestTrain,options,fc);

size(newMatrix)
name="datasetMarco"
save ((name),"newMatrix")






