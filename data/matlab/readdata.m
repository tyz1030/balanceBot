sta20 = csvread('pwm_20_stable.csv');
sta40 = csvread('pwm_40_stable.csv');
sta60 = csvread('pwm_60_stable.csv');
sta80 = csvread('pwm_80_stable.csv');
sta100 = csvread('pwm_100_stable.csv');

sta20L = sta20(181:2:5565,:);
sta20R = sta20(182:2:5566,:);

sta40L = sta40(181:2:2553,:);
sta40R = sta40(182:2:2554,:);

sta60L = sta60(181:2:1451,:);
sta60R = sta60(182:2:1452,:);

sta80L = sta80(181:2:983,:);
sta80R = sta80(182:2:984,:);

sta100L = sta100(181:2:755,:);
sta100R = sta100(182:2:756,:);

clear sta20 sta40 sta60 sta80 sta100
%%reshape data
sta20L = sta20L(1:200,:);
sta20R = sta20R(1:200,:);
sta40L = sta40L(1:200,:);
sta40R = sta40R(1:200,:);
sta60L = sta60L(1:200,:);
sta60R = sta60R(1:200,:);
sta80L = sta80L(1:200,:);
sta80R = sta80R(1:200,:);
sta100L = sta100L(1:200,:);
sta100R = sta100R(1:200,:);

sta = zeros(200,8,10);
sta(:,1:4,1) = sta20L;
sta(:,1:4,2) = sta20R;
sta(:,1:4,3) = sta40L;
sta(:,1:4,4) = sta40R;
sta(:,1:4,5) = sta60L;
sta(:,1:4,6) = sta60R;
sta(:,1:4,7) = sta80L;
sta(:,1:4,8) = sta80R;
sta(:,1:4,9) = sta100L;
sta(:,1:4,10) = sta100R;

clear sta100L sta100R sta80L sta80R sta60L sta60R sta40L sta40R sta20L sta20R