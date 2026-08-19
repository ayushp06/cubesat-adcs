function generateCppReferenceVectors(outputFile)
% GENERATECPPREFERENCEVECTORS Deterministic MATLAB outputs consumed by C++ tests.
    testDir=fileparts(mfilename("fullpath")); addpath(genpath(fullfile(testDir,"..")));
    outputDir=fileparts(outputFile); if ~exist(outputDir,"dir"),mkdir(outputDir);end
    file=fopen(outputFile,"w");
    q1=quatNormalize([.9;.1;-.2;.3]);q2=quatNormalize([.8;-.3;.1;.2]);
    row(file,"multiply",quatMultiply(q1,q2)); dcm=quatToDCM(q1); row(file,"dcm",dcm(:));

    p=mekfParams();x=mekfInitialize([1;0;0;0],[.001;-.002;.0005],p);
    x=mekfPredict(x,[.01;-.02;.03],.01,p);
    row(file,"mekf_predict_q",x.qIB);row(file,"mekf_predict_bias",x.gyroBias);row(file,"mekf_predict_pdiag",diag(x.covariance));
    x=mekfUpdateVectors(x,[.3;-.4;.866025403784],[.2;-.1;.974679434481],.008);
    row(file,"mekf_update_q",x.qIB);row(file,"mekf_update_bias",x.gyroBias);row(file,"mekf_update_pdiag",diag(x.covariance));row(file,"mekf_update_nis",x.lastNIS);

    sun=[1;2;3]/sqrt(14);row(file,"sun",sunPointing(sun));
    row(file,"nadir",nadirPointing([7e6;1e5;-2e5],[-100;7500;900]));
    [qSlew,wSlew]=quaternionSlewReference([1;0;0;0],[sqrt(.5);0;0;sqrt(.5)],3,10);
    row(file,"slew_q",qSlew);row(file,"slew_rate",wSlew);

    sc=spacecraftParams();controller=controllerParams(sc);lqr=lqrControllerParams(sc);
    qRef=[sqrt(.5);sqrt(.5);0;0];qEst=quatNormalize([.98;.1;-.05;.02]);omega=[.01;-.02;.03];
    row(file,"pd",quaternionPDController(qRef,qEst,omega,controller));
    row(file,"lqr",quaternionLqrController(qRef,qEst,omega,lqr));
    rw=reactionWheelParams();row(file,"allocation",allocateReactionWheelTorque([1e-4;-2e-4;5e-5],rw));

    state=initializeAdcsMode();s=struct("fault",false,"faultReset",false,"initializationComplete",true,"bodyRate",.1,"safeRequested",false,"estimatorValid",true,"wheelSpeedFraction",0,"slewRequested",false,"slewComplete",false,"nominalRequested",true);m=zeros(5,1);
    state=updateAdcsMode(state,s,modeManagerParams());m(1)=modeNumber(state.mode);s.bodyRate=.001;state=updateAdcsMode(state,s,modeManagerParams());m(2)=modeNumber(state.mode);state=updateAdcsMode(state,s,modeManagerParams());m(3)=modeNumber(state.mode);s.wheelSpeedFraction=.9;state=updateAdcsMode(state,s,modeManagerParams());m(4)=modeNumber(state.mode);s.fault=true;state=updateAdcsMode(state,s,modeManagerParams());m(5)=modeNumber(state.mode);row(file,"modes",m);
    fclose(file);
end
function row(file,name,values)
    fprintf(file,"%s",name);fprintf(file,",%.17g",values);fprintf(file,"\n");
end
function number=modeNumber(mode)
    names={"initialization","detumble","safe","nominal","slew","desaturation","fault"};number=find(strcmp(names,mode),1)-1;
end
