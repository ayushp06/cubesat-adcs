function [gain,riccati] = continuousLqr(A,B,Q,R)
% CONTINUOUSLQR Solve continuous CARE from the Hamilton stable subspace.
    n=size(A,1); H=[A,-B*(R\B');-Q,-A'];
    [vectors,values]=eig(H); stable=find(real(diag(values))<0);
    if numel(stable)~=n, error("continuousLqr:NoStableSubspace","CARE has no unique stabilizing solution."); end
    U=vectors(:,stable); riccati=real(U(n+1:end,:)/U(1:n,:));
    riccati=(riccati+riccati')/2; gain=R\(B'*riccati);
end
