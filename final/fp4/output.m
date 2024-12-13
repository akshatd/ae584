function y = output(x)
R    = sqrt( (x(7 )-x(3 ))^2 +  (x(8 )-x(4 ))^2  );
Rdot = ( (x(7 )-x(3 ))*(x(5 )*sin(x(6 )) - x(1 )*sin(x(2 )))  +  (x(8 )-x(4 ))*( x(5 )*cos(x(6 )) - x(1 )*cos(x(2 )) )  )/R;
betadot =  (   (x(8 )-x(4 ))*( x(5 )*sin(x(6 )) - x(1 )*sin(x(2 )) ) -  (x(7 )-x(3 ))*(x(5 )*cos(x(6 )) - x(1 )*cos(x(2 )))    )/ ( (x(7 )-x(3 ))^2 +  (x(8 )-x(4 ))^2  );
y = [R;Rdot;betadot];
end

