

%%
subscriber = rossubscriber("floats","DataFormat","struct");
while true
vector=zeros(1,64,1536);
data=receive(subscriber);
vector(1,:,:)=reshape(data.Data,[64,1536]);
% for i=1:1536
%     data=receive(subscriber);
%     vector(1,:,i)=data.Data;
% end
fc=512;
Ts=1/fc;

%% first preprocessing
importantChannels=[14,13,12,10,18,48,49,50];%,46,56];
%x=[epoch_number,channels,samples]

epoch_size=1536;

vector=preprocessingFirstStep(vector,importantChannels);
vector=reshape(vector,[length(importantChannels),epoch_size,1]);


newMatrix=zeros(1,8,3,32);
%x=[channels,samples,epoch_number]
newMatrix(1,:,:,:)=preprocessingSecondStep(vector,true,fc);
chatterpub = rospublisher("elaborated","rospy_tutorials/Floats","DataFormat","struct");

elaborated = rosmessage(chatterpub);
elaborated.Data = single(newMatrix);%reshape(newMatrix,[960,1]);
send(chatterpub,elaborated)


end
