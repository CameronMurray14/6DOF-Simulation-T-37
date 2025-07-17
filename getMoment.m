function [moment] = getMoment(ipl,dynPress,alpha,alphaDot,beta,params,controls)
%% Compute moments in body (vehicle) frame
%global params;
wingSpan = params.wingSpan; % feet
cbar = params.cbar; %feet
Sref = params.Sref; %square feet 
Cmq = params.Cmq;
Cm0 = params.Cm0;
CmAlpha = params.CmAlpha;
CmAlphaDot = params.CmAlphaDot;
CmDelta_e = params.CmDelta_e;
Croll0 = params.Croll0;
CrollBeta = params.CrollBeta;
CrollP = params.CrollP;
CrollR = params.CrollR;
CrollDeltaA = params.CrollDeltaA;
CrollDeltaR = params.CrollDeltaR;
Cyaw0 = params.Cyaw0;
CnBeta = params.CnBeta;
CnDeltaA = params.CnDeltaA;
CnDeltaR = params.CnDeltaR;
CnR = params.CnR;
CnP = params.CnP;
%% Compute pitching moment
delta_e = controls.delta_e;
Cm = Cm0 + CmAlpha*alpha + CmAlphaDot*alphaDot + Cmq*ipl(8) + CmDelta_e*delta_e;
momentY = Cm * dynPress * cbar * Sref;
%% Compute rolling moment
delta_a = controls.delta_a;
delta_r = controls.delta_r;
Croll = Croll0 + CrollBeta*beta + CrollP*ipl(7) + CrollR*ipl(9) + CrollDeltaA*delta_a + CrollDeltaR*ipl(9);
momentX = Croll * dynPress * wingSpan * Sref;
%% Compute yawing moment
Cyaw = Cyaw0 + CnBeta*beta + CnDeltaA*delta_a + CnDeltaR*delta_r + CnR*ipl(9) + CnP*ipl(7);
momentZ = Cyaw * dynPress * wingSpan * Sref;
%% Moment vector
moment = [momentX; momentY; momentZ];
end