



function [s,xt] = LocalGetInfo(s,xt,y,t,yf,SettlingTimeThreshold,RiseTimeLims,Ts)
if isfinite(yf)
   % Peak response
   [s.Peak,ipeak] = max(abs(y));
    s.PeakTime = t(ipeak);

end
