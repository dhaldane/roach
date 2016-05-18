fmax = pi/2;
resolution = 2^8;
femur = linspace(0,fmax,resolution)
crank = 2.003.*femur.^5.0 - 7.434.*femur.^4.0 + 9.429.*femur.^3.0 - 2.691.*femur.^2.0 + 0.3893.*femur - 0.001175;
scale = floor((2^16-200)/crank(end));
crank = floor(max(0,crank.*scale));

crScale = 2*pi*scale/(25*2^16);

header = 'static unsigned int crank_femur_256lut[256] = {\n';
lutString = sprintf('%u,' , crank);
lutString = lutString(1:end-1);% strip final comma
footer = '\n};\n';

fid = fopen('../lib/lut.h','wt');
fprintf(fid, header);
fprintf(fid, lutString);
fprintf(fid, footer);
fprintf(fid, ['static float motPos_to_femur_crank_units = ' num2str(crScale) ';\n']);
fclose(fid);