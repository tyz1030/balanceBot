V = 1.2*[2 4 6 8 10]';
RL = 0.44;
RR = 0.55;

iL = avei(1:2:9);
iR = avei(2:2:10);

wL = avew(1:2:9);
wR = avew(2:2:10);

KL = (V-iL*RL)./wL;
KR = (V-iR*RR)./wR;

%average K
KL(6) = mean(KL(1:5));
KR(6) = mean(KR(1:5));
