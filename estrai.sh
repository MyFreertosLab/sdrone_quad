echo RC='['
cat trace-volo-11.txt|grep rc|sed -e 's/^rc..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo RPY='['
cat trace-volo-11.txt|grep rpy|sed -e 's/^rpy..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo ACC='['
cat trace-volo-11.txt|grep acc|sed -e 's/^acc..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo GR='['
cat trace-volo-11.txt|grep gr|sed -e 's/^gr..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo AX='['
cat trace-volo-11.txt|grep ax|sed -e 's/^ax..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo W='['
cat trace-volo-11.txt|grep w|sed -e 's/^w..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo VV='['
cat trace-volo-11.txt|grep vv|sed -e 's/^vv..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
cat <<EOF
RC_THRUST=RC(1:size(RC,1),2);
RC_ROLL=RC(1:size(RC,1),3);
RC_PITCH=RC(1:size(RC,1),4);
RC_YAW=RC(1:size(RC,1),5);

ACC_X=ACC(1:size(ACC,1),2);
ACC_Y=ACC(1:size(ACC,1),3);
ACC_Z=ACC(1:size(ACC,1),4);

W_X=W(1:size(W,1),2);
W_Y=W(1:size(W,1),3);
W_Z=W(1:size(W,1),4);
W_THRUST=W(1:size(W,1),5);

AX_X=AX(1:size(AX,1),2);
AX_Y=AX(1:size(AX,1),4);
AX_Z=AX(1:size(AX,1),3);
AX_T=AX(1:size(AX,1),5);

GR=GR .* (-1);
GR_X=GR(1:size(GR,1),2);
GR_Y=GR(1:size(GR,1),3);
GR_Z=GR(1:size(GR,1),4);

RPY_X=RPY(1:size(RPY,1),2);
RPY_Y=RPY(1:size(RPY,1),3);
RPY_Z=RPY(1:size(RPY,1),4);

VV_V=VV(1:size(VV,1),2);

function plotRC();
  subplot(231);
  //scf();
  plot(RC_THRUST, "color", "black");
  plot(RC_ROLL,"color", "red");
  plot(RC_PITCH,"color", "blue");
  plot(RC_YAW,"color", "cyan");
  xtitle("RC")
endfunction

function plotACC();
  subplot(232);
  //scf();
  plot(ACC_X,"color", "red");
  plot(ACC_Y,"color", "blue");
  plot(ACC_Z,"color", "black");
  xtitle("Accelerometer")
endfunction

function plotW();
  subplot(233);
  //scf();
  plot(W_THRUST, "color", "black");
  plot(W_X,"color", "blue");
  plot(W_Y,"color", "red");
  plot(W_Z,"color", "black");
  xtitle("W")
endfunction

function plotVV();
  subplot(233);
  //scf();
  plot(VV_V, "color", "black");
  xtitle("Vertical Speed")
endfunction

function plotAX();
  subplot(234);
  //scf();
  plot(AX_X,"color", "red");
  plot(AX_Y,"color", "blue");
  plot(AX_Z,"color", "black");
  plot(AX_T,"color", "black");
  xtitle("AX")
endfunction

function plotGR();
  subplot(235);
  //scf();
  plot(GR_X,"color", "red");
  plot(GR_Y,"color", "blue");
  plot(GR_Z,"color", "black");
  xtitle("GR")
endfunction

function plotRPY();
  subplot(236);
  //scf();
  plot(RPY_X,"color", "red");
  plot(RPY_Y,"color", "blue");
  plot(RPY_Z,"color", "black");
  xtitle("RPY")
endfunction


function R = mroll(r);
  R=[1,0,0;0,cos(r),-sin(r);0, sin(r), cos(r)];
endfunction
function R = mpitch(r);
  R=[cos(r),0,sin(r);0,1,0;-sin(r),0,cos(r)];
endfunction
function R = myaw(r);
  R=[cos(r),-sin(r),0;sin(r),cos(r),0;0,0,1];
endfunction
function R = mrpy(r);
  R = myaw(r(3))*mpitch(r(2))*mroll(r(1));
endfunction
function R = mrp(r);
  R = mpitch(r(2))*mroll(r(1));
endfunction
function V = rot(X,R);
  V=[0];
  s = min(size(X,1),size(R,1));
  for i=1:s
     V(i,1:3) = (mrpy(R(i,1:3)')*(X(i,1:3)'))'
  end
endfunction
function V = invRot(X,R);
  V=[0];
  s = min(size(X,1),size(R,1));
  for i=1:s
     V(i,1:3) = (inv(mrpy(R(i,1:3)'))*(X(i,1:3)'))'
  end
endfunction
function V = rotRP(X,R);
  V=[0];
  s = min(size(X,1),size(R,1));
  for i=1:s
     V(i,1:3) = (mrp(R(i,1:3)')*(X(i,1:3)'))'
  end
endfunction
function V = accIf(A,G,R);
  s = min(min(size(A,1),size(G,1)), size(R,1));
  V = rotRP(A(1:s,1:3)-G(1:s,1:3),R(1:s,1:3))
endfunction

function V = accIf2(A,R);
  s = min(size(A,1), size(R,1));
  G=[];
  G(1:s,1)=0;
  G(1:s,2)=0;
  G(1:s,3)=-9.8;
  V = invRot(rot(A(1:s,1:3),R(1:s,1:3))+G(1:s,1:3),R(1:s,1:3))
endfunction

s = min(min(size(ACC,1),size(GR,1)),size(RPY,1));
//ACC_IF=accIf(ACC(1:s,2:4), GR(1:s,2:4), RPY(1:s,2:4));
ACC_IF=accIf2(ACC(1:s,2:4), RPY(1:s,2:4));
ACC_IF_X=ACC_IF(1:size(ACC_IF,1),1);
ACC_IF_Y=ACC_IF(1:size(ACC_IF,1),2);
ACC_IF_Z=ACC_IF(1:size(ACC_IF,1),3);

function plotACC_IF();
  scf();
  plot(ACC_IF_X,"color", "red");
  plot(ACC_IF_Y,"color", "blue");
  plot(ACC_IF_Z,"color", "black");
  xtitle("ACC_IF")
endfunction

plotRC();
plotACC();
//plotW();
plotVV();
plotAX();
plotGR();
plotRPY();
plotACC_IF();
EOF

