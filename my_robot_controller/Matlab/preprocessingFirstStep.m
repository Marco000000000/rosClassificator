function Signal=preprocessingFirstStep(x,importantChannels)%x=[epoch_number,channels,samples]
%% filtering
epoch_number=size(x,1)
channels=size(x,2)
epoch_size=size(x,3)

initialSize=size(x);
%filtering 8-30 Hz
temp_sign=permute(x,[1 3 2]);
temp_sign=reshape(temp_sign,[epoch_number,channels*epoch_size]);





for e=1:1:epoch_number
    temp_sign(e,:)=filter(filtroPB,temp_sign(e,:));

%     for i=1:1:channels
%         temp_sign(e,i,:)=filter(filtroPB,x(e,i,:));
%     end
end
temp_sign=reshape(temp_sign,[initialSize(1),initialSize(3),initialSize(2)]);
temp_sign=permute(temp_sign,[1 3 2]);

for e=1:1:epoch_number
    for i=1:channels
    temp_sign(e,i,:)=temp_sign(e,i,:)-mean(temp_sign(e,i,:));
    end
end


%% common average


for e=1:1:epoch_number

    for i =1:epoch_size
        for c= importantChannels
        temp_sign(e,c,i)=temp_sign(e,c,i)-mean(temp_sign(e,:,i));
    
        end
    end
end
Signal=temp_sign(:,importantChannels,:);

end