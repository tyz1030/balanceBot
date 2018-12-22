inl = csvread('data_pwm_100_inertia_L.csv');
inr = csvread('data_pwm_100_inertia_R.csv');

inel = inl(181:2:2487,:);
iner = inr(182:2:3102,:);

clear inl inr

inel(:,2) = -inel(:,2);

inel(:,1) = inel(:,1)-inel(1,1);
iner(:,1) = iner(:,1)-iner(1,1);

ine = cat(3,inel(1:1000,:),iner(1:1000,:));

estine = ine(301:400,:,:);
%reset time
estine(:,1,:) = (estine(:,1,:)-estine(1,1,:))./1e6;
tl = estine(:,1,1);
tr = estine(:,1,2);

Al = [ones(100,1) tl tl.^2 tl.^3];
Ar = [ones(100,1) tr tr.^2 tr.^3];

dAl = [zeros(100,1) ones(100,1) tl tl.^2];
dAr = [zeros(100,1) ones(100,1) tr tr.^2];

ddAl = [zeros(100,1) zeros(100,1) ones(100,1) tl];
ddAr = [zeros(100,1) zeros(100,1) ones(100,1) tr];

pl = pinv(Al)*estine(:,2,1);
pr = pinv(Ar)*estine(:,2,2);

estine(:,5,1) = Al*pl;
estine(:,5,2) = Ar*pr;

estine(:,6,1) = dAl*pl;
estine(:,6,2) = dAr*pr;

estine(:,7,1) = ddAl*pl;
estine(:,7,2) = ddAr*pr;
%reshape
%caculate count differ
% ine(2:end,5:6,:) = ine(2:end,1:2,:)-ine(1:end-1,1:2,:);
%caculate w
% ine(2:end,7,:) = (ine(2:end,6,:)./ine(2:end,5,:))/976*2*pi*1e6;

%smoothing
% ine(2:end-1,7,:) = (ine(1:end-2,7,:)+ine(2:end-1,7,:)+ine(3:end,7,:))/3;
% ine(2:end-1,7,:) = (ine(1:end-2,7,:)+ine(2:end-1,7,:)+ine(3:end,7,:))/3;

%caculate dw/dt
% ine(2:end,8,:) = 1e6*(ine(2:end,7,:)-ine(1:end-1,7,:))./ine(2:end,5,:);

%smoothing
% ine(2:end-1,8,:) = (ine(1:end-2,8,:)+ine(2:end-1,8,:)+ine(3:end,8,:))/3;
% ine(2:end-1,8,:) = (ine(1:end-2,8,:)+ine(2:end-1,8,:)+ine(3:end,8,:))/3;
% ine(2:end-1,8,:) = (ine(1:end-2,8,:)+ine(2:end-1,8,:)+ine(3:end,8,:))/3;
clear inel iner

jl = -([estine(:,6,1) ones(100,1)]*bcl)./estine(:,7,1);
jr = -([estine(:,6,2) ones(100,1)]*bcr)./estine(:,7,2);

% jbcl = pinv([estine(:,7,1) estine(:,6,1) ones(100,1)])*zeros(100,1);
% jbcr = pinv([estine(:,7,2) estine(:,6,2) ones(100,1)])*zeros(100,1);

