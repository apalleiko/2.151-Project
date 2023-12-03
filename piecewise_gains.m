function Kt = piecewise_gains(K,times,t)
i = times <= t;
Kti = K(:,:,i);
Kt = Kti(:,:,end);
end

