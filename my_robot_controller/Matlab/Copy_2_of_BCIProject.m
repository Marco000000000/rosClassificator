

%%
subscriber = rossubscriber("floats","DataFormat","struct");
while true
vector=zeros(1,14,1536);
data=receive(subscriber);
for i=1:2:1536
    data=receive(subscriber);
    vector(1,:,i)=data.Data;
    vector(1,:,i+1)=data.Data;
end
fc=512;
Ts=1/fc;

%% first preprocessing
importantChannels=1:14;%,46,56];
%x=[epoch_number,channels,samples]

epoch_size=1536;
vector=reshape(vector,[14,1536,1]);
vector=preprocessingFirstStep(vector,importantChannels);
vector=reshape(vector,[length(importantChannels),epoch_size,1]);
options=[false,true];


newMatrix=zeros(1,14,32);
%x=[channels,samples,epoch_number]
newMatrix(1,:,:)=preprocessingSecondStep(vector,options,fc);
chatterpub = rospublisher("elaborated","rospy_tutorials/Floats","DataFormat","struct");

elaborated = rosmessage(chatterpub);
elaborated.Data = single(newMatrix);%reshape(newMatrix,[960,1]);
send(chatterpub,elaborated)


end
