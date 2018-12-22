bcl = pinv(cat(2,wL(1:4),ones(4,1)))*torl(1:4);
bcr = pinv(cat(2,wR(1:4),ones(4,1)))*torr(1:4);
