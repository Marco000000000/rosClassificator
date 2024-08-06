function Signal=preprocessingSecondStep(x,options,fc)%x=[channels,samples,epoch_number]
    channels=size(x,1);
    epoch_size=size(x,2);
    epoch_number=size(x,3);
    
    Signal= zeros(epoch_number,channels,3,epoch_size);
    if(options(1)==true)
        for i = 1:epoch_number
           for e=1:channels
               try1=itd(x(e,:,i));
               Signal(i,e,:,:)=try1(1:3,:);
           end
        end
    else
        Signal= reshape(x,[epoch_number,channels,1,epoch_size]);
    end

    if options(2)==true
        newMatrix=zeros(size(Signal,1),size(Signal,2),size(Signal,3),32);
         for a =1:(size(Signal,1))
                for i= 1:size(Signal,2)
                    for o=1:size(Signal,3)
                        %size(Signal(a,i,o,:))
                        temp=reshape(Signal(a,i,o,:),[1,size(Signal,4)]);
                        newMatrix(a,i,o,1)=max(xcorr(temp,temp));
                        newMatrix(a,i,o,2)=bandpower(temp);
                        %entropia instantaneous è una variabile che permette di
                        %inibire l'operazione di media sulle varie frequenze
                        newMatrix(a,i,o,3:end)=pentropy(temp,fc,'FrequencyLimits',[8 30],'Instantaneous', true);
       
                    end
                end
         end
         if options(1)==false
            newMatrix=reshape(newMatrix,[size(Signal,1),size(Signal,2),32]);
         end
         size(newMatrix)
         Signal=newMatrix;
    end
    
end



