
function scrollPlot(devname)

 if nargin <1; devname = '/dev/ttyACM0'; end

 fp = fopen(devname);

 sampSincePlot=0;

 newdata = zeros(100,1);
 alldata = zeros(1000,1);

 while(true);
   while (sampSincePlot<100);
	sampSincePlot = sampSincePlot + 1;
	%disp(num2str(sampSincePlot));
	str = fgets(fp);
	newdata(sampSincePlot,1) = str2num(str);
   end
   sampSincePlot=0;

   alldata(1:900,1) = alldata(101:end,1);
   alldata(901:end,1) = newdata;

   plot(1:numel(alldata), alldata);
   pause(0.005);
 end


 fclose(devname);

end

