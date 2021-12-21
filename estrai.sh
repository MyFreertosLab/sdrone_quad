source_file=$1
echo urot='['
cat $source_file|grep -e '^urot:'|sed -e 's/^urot:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo rpy='['
cat $source_file|grep -e '^rpy:'|sed -e 's/^rpy:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo alfa='['
cat $source_file|grep -e '^alfa:'|sed -e 's/^alfa:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo ealfa='['
cat $source_file|grep -e '^ealfa:'|sed -e 's/^ealfa:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo uacc='['
cat $source_file|grep -e '^uacc:'|sed -e 's/^uacc:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo speed='['
cat $source_file|grep -e '^speed:'|sed -e 's/^speed:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo acc='['
cat $source_file|grep -e '^acc:'|sed -e 's/^acc:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
echo eacc='['
cat $source_file|grep -e '^eacc:'|sed -e 's/^eacc:..*\[//g'|sed -e 's/\]/;/g'|sed -e 's/ /,/g'
echo '];'
cat <<EOF

urot_roll=urot(1:size(urot,1),2);
urot_pitch=urot(1:size(urot,1),3);
urot_yaw=urot(1:size(urot,1),4);

rpy_roll=rpy(1:size(rpy,1),2);
rpy_pitch=rpy(1:size(rpy,1),3);
rpy_yaw=rpy(1:size(rpy,1),4);

alfa_roll=alfa(1:size(alfa,1),2);
alfa_pitch=alfa(1:size(alfa,1),3);
alfa_yaw=alfa(1:size(alfa,1),4);

ealfa_roll=ealfa(1:size(ealfa,1),2);
ealfa_pitch=ealfa(1:size(ealfa,1),3);
ealfa_yaw=ealfa(1:size(ealfa,1),4);

uacc_x=uacc(1:size(uacc,1),2);
uacc_y=uacc(1:size(uacc,1),3);
uacc_z=uacc(1:size(uacc,1),4);

speed_x=speed(1:size(speed,1),2);
speed_y=speed(1:size(speed,1),3);
speed_z=speed(1:size(speed,1),4);

acc_x=acc(1:size(acc,1),2);
acc_y=acc(1:size(acc,1),3);
acc_z=acc(1:size(acc,1),4);

eacc_x=eacc(1:size(eacc,1),2);
eacc_y=eacc(1:size(eacc,1),3);
eacc_z=eacc(1:size(eacc,1),4);

function plotROLL();
  subplot(231);
  plot(urot_roll, "color", "black");
  plot(rpy_roll,"color", "red");
  xtitle("ROLL")
endfunction

function plotPITCH();
  subplot(232);
  plot(urot_pitch, "color", "black");
  plot(rpy_pitch,"color", "red");
  xtitle("PITCH")
endfunction

function plotYAW();
  subplot(233);
  plot(urot_yaw, "color", "black");
  plot(rpy_yaw,"color", "red");
  xtitle("YAW")
endfunction

function plotALFA_ROLL();
  subplot(234);
  plot(alfa_roll, "color", "black");
  plot(ealfa_roll,"color", "red");
  xtitle("ALFA_ROLL")
endfunction

function plotALFA_PITCH();
  subplot(235);
  plot(alfa_pitch, "color", "black");
  plot(ealfa_pitch,"color", "red");
  xtitle("ALFA_PITCH")
endfunction

function plotALFA_YAW();
  subplot(236);
  plot(alfa_yaw, "color", "black");
  plot(ealfa_yaw,"color", "red");
  xtitle("ALFA_YAW")
endfunction

function plotACCX();
  subplot(311);
  plot(uacc_x, "color", "black");
  plot(acc_x,"color", "red");
  plot(eacc_x,"color", "cyan");
  xtitle("ACCX")
endfunction

function plotACCY();
  subplot(312);
  plot(uacc_y, "color", "black");
  plot(acc_y,"color", "red");
  plot(eacc_y,"color", "cyan");
  xtitle("ACCY")
endfunction

function plotACCZ();
  subplot(313);
  plot(uacc_z, "color", "black");
  plot(acc_z,"color", "red");
  plot(eacc_z,"color", "cyan");
  xtitle("ACCZ")
endfunction

function plotSPEED();
  subplot(211);
  plot(speed_x, "color", "green");
  plot(speed_y,"color", "red");
  plot(speed_z,"color", "black");
  xtitle("SPEED")
endfunction

plotROLL();
plotPITCH();
plotYAW();
plotALFA_ROLL();
plotALFA_PITCH();
plotALFA_YAW();
scf();
plotACCX();
plotACCY();
plotACCZ();
scf();
plotSPEED();

EOF

