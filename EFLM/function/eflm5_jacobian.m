function J = eflm5_jacobian(q,Y,G,p)
% Batched derivative of the unreduced three angular equations.
N=size(q,1); J=zeros(3,3,N); epsc=1e-20;
for j=1:3
 qc=q; qc(:,j)=qc(:,j)+1i*epsc;
 e=eflm5_balance(qc,Y,G,p);
 J(:,j,:)=reshape((imag(e)/epsc)',3,1,N);
end
end
