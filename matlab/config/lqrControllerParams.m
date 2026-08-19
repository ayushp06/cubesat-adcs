function controller = lqrControllerParams(sc)
% LQRCONTROLLERPARAMS Gain and analytical diagnostics for linear attitude error.
    [A,B]=linearizedAttitudeModel(sc.J);
    Q=diag([ones(1,3),0.1*ones(1,3)]); R=4e6*eye(3);
    [gain,P]=continuousLqr(A,B,Q,R);
    controllability=B;
    for power=1:5, controllability=[controllability,A^power*B]; end
    controller.gain=gain; controller.Q=Q; controller.R=R; controller.P=P;
    controller.A=A; controller.B=B;
    controller.controllabilityRank=rank(controllability);
    controller.closedLoopEigenvalues=eig(A-B*gain);
end
