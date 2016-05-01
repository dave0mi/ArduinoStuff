
function scrollPlot(devname)

 if nargin <1; devname = '/dev/ttyACM0'; end

 fp = fopen(devname);

 sampSincePlot=0;

 newdata = zeros(128,1);
 alldata = zeros(1024,1);
 avgfft  = [];

 while(true);
   while (sampSincePlot<128);
	sampSincePlot = sampSincePlot + 1;
	%disp(num2str(sampSincePlot));
	str = fgets(fp);
	num = str2num(str);
	%if (strcmp(class(num),'double') && (numel(num) == 1))
	  newdata(sampSincePlot,1) = num;
	%end
   end
   sampSincePlot=0;

   alldata(1:(end-127),1)   = alldata(128:end,1);
   alldata((end-127):end,1) = newdata;

   %plot(h1, 1:numel(alldata), alldata);
   figure(1);
   plot(1:numel(alldata), alldata);
   set(gca, 'YLim', [0 780]);

   figure(2);
   freqres = fft(alldata);
   freqres = real(freqres(2:257));

%disp(num2str(max(freqres)));
%   freqres = freqres./max(freqres);

   if (isempty(avgfft)); avgfft = freqres; else; avgfft = 0.5.*avgfft+0.5*freqres; end
   %%% don't plot DC offset or most higher frequencies ...
   %plot(1:numel(freqres), freqres);
   plot(1:numel(avgfft), avgfft);
   %set(gca, 'YLim', [-1 1]);
   set(gca, 'YLim', [-16000 16000]);

   pause(0.005);
 end


 fclose(devname);

end

