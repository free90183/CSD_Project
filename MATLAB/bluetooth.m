bt = Bluetooth('Team17Slave',1);
fopen(bt);

data = zeros(500,2);
for i = 1:500
   data(i,1) = str2double(fscanf(bt));
   data(i,2) = str2double(fscanf(bt));
end

fclose(bt);
