%set time begin with 0
sta(:,1,:) = sta(:,1,:)-sta(1,1,:);
%set L positive count
sta(:,2,:) = abs(sta(:,2,:));
%caculate count differ
sta(2:end,5:6,:) = sta(2:end,1:2,:)-sta(1:end-1,1:2,:);
%caculate w
sta(2:end,7,:) = (sta(2:end,6,:)./sta(2:end,5,:))/976*2*pi*1e6;  
%average i and w
sta(201,[3 7],:) = mean(sta(1:200,[3 7],:));
avei = reshape(sta(201,3,:),10,1);
avew = reshape(sta(201,7,:),10,1);