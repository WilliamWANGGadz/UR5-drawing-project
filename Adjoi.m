function Adj=Adjoi(Tr)
%gives the adjoint of the homogenous representation
R=[Tr(1,1),Tr(1,2),Tr(1,3);Tr(2,1),Tr(2,2),Tr(2,3);Tr(3,1),Tr(3,2),Tr(3,3)];
p=[Tr(1,4);Tr(2,4);Tr(3,4)];

Pskew=[0,-p(3),p(2);p(3),0,-p(1);-p(2),p(1),0];

Adj=horzcat(R,Pskew*R);
Adj=vertcat(Adj,horzcat(zeros(3,3),R));

end