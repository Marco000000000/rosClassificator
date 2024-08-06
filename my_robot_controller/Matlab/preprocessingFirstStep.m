function Signal=preprocessingFirstStep(x,importantChannels)%x=[channels,epoch_size,epoch_number]
%% filtering
channels=size(x,1)
epoch_size=size(x,2)
epoch_number=size(x,3)

initialSize=size(x);
%filtering 8-30 Hz
temp_sign=permute(x,[1 3 2]);
temp_sign=reshape(x,[channels,epoch_number*epoch_size]);




for e=1:1:channels
    temp_sign(e,:)=filter(filtroPB,temp_sign(e,:));


end

for i=1:channels
    temp_sign(i,:)=temp_sign(i,:)-mean(temp_sign(i,:));
end



%% common average
a= temp_sign;
num=epoch_size*epoch_number;
for i =1:num
    for e= importantChannels

        % temp_sign(e,i)=temp_sign(e,i)-mean(temp_sign(importantChannels,i));
        %temp_sign(e,i)=temp_sign(e,i)-(mean(temp_sign(importantChannels,i)*length(importantChannels)-temp_sign(e,i))/(length(importantChannels)-1));
        a(e,i)=temp_sign(e,i)-(mean(temp_sign(importantChannels,i)*length(importantChannels)-temp_sign(e,i))/(length(importantChannels)-1));

    end
end
temp_sign=a;
temp_sign=reshape(temp_sign,[channels,epoch_size,epoch_number]);

% for e=1:1:epoch_number
% 
%     for i =1:epoch_size
%         for c= importantChannels
%             temp_sign(e,c,i)=temp_sign(e,c,i)-(mean(temp_sign(e,:,i)*epoch_number-temp_sign(e,c,i))/(epoch_number-1));
%         end
%     end
% end
Signal=temp_sign(importantChannels,:,:);

end