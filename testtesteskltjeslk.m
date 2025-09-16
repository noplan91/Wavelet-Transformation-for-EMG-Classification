%%wavelets:
db4 = 'db4';
db8 = 'db8';
db16 = 'db16';
db32 = 'db32';
db64 = 'db64';
%%set wavelet
str = [db16, db4, db8, db32, db64];
wname = ["db4","db8","db16","db32","db64","bior1.5"];
wname1 = wname(1);
num_tries = 1;
for index = 1:1:num_tries
wname = wname(num_tries);
end

out_halllo = [1, 4 , 6, wname1]
    