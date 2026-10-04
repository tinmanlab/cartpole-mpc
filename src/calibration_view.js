// View-only connection to CalibrationLab. No alternate simulator/filter or auto-promotion.
const CalibrationLessonView={mount({manifest,pause}){
 const C=globalThis.CalibrationLab,$=id=>document.getElementById(id),m=manifest;
 let status='idle',lastError=null,busy=false,runs=[],fit=null,capture=null,replay=[],caseId=m.cases[0].id,armId=m.arms[0].id,index=0;
 const number=(v,d=3)=>Number.isFinite(v)?v.toFixed(d):'—';
 const text=(id,s)=>$(id).textContent=s;
 $('calibrationCase').innerHTML=m.cases.map(c=>'<option value="'+c.id+'">'+c.id+' · '+(c.group==='primary'?'독립 잡음':'경계: '+c.scenario)+'</option>').join('');
 $('calibrationArm').innerHTML=m.arms.map(a=>'<option value="'+a.id+'">'+a.label+'</option>').join('');
 const chosen=()=>runs.find(r=>r.armId===armId);
 function getState(){const r=chosen();return {status,lastError,caseId,armId,index,fit,frame:r?.trace[index]??null,runs:runs.map(({trace,...s})=>s),sameDataReplay:replay,publicDefaultsChanged:false};}
 function histogram(){
  const g=$('calibrationHistogram').getContext('2d'),W=900,H=170;g.clearRect(0,0,W,H);if(!fit)return;
  const labels=['p − sample mean [m]','θ − sample mean [rad]'];
  for(let channel=0;channel<2;channel++){
   const left=channel*450+45,width=350,base=138,limit=4*fit.standardDeviation[channel],counts=Array(24).fill(0);
   for(const y of capture.measurements){const d=y[channel]-fit.mean[channel],bin=Math.max(0,Math.min(23,Math.floor((d+limit)/(2*limit)*24)));counts[bin]++;}
   const high=Math.max(...counts);g.fillStyle='#506679';g.font='15px system-ui';g.fillText(labels[channel]+' · count',left,22);
   counts.forEach((n,i)=>{const h=n/high*90;g.fillStyle=channel?'#936323':'#2c74a5';g.fillRect(left+i*width/24,base-h,width/24-2,h);});
   g.fillStyle='#506679';g.fillText(number(-limit,3),left,160);g.fillText('0',left+width/2,160);g.fillText(number(limit,3),left+width-48,160);
  }
 }
 function timeline(){
  const canvas=$('calibrationTimeline'),g=canvas.getContext('2d'),W=1100,H=350,run=chosen();g.clearRect(0,0,W,H);canvas.dataset.units='m,deg,N';if(!run)return;
  const all=runs.flatMap(r=>r.trace),left=75,right=W-25,lastTime=Math.max(...runs.map(r=>r.appliedSteps))*.02;
  const bands=[{label:'위치 [m]',series:[{f:r=>r.truth[0],name:'truth',color:'#276eab'},{f:r=>r.estimate[0],name:'estimate',color:'#ac7521'},{f:r=>r.goal,name:'goal',color:'#688458'}]},
   {label:'막대 각도 [deg]',series:[{f:r=>r.truth[2]*180/Math.PI,name:'truth',color:'#276eab'},{f:r=>r.estimate[2]*180/Math.PI,name:'estimate',color:'#ac7521'}]},
   {label:'힘 [N]',series:[{f:r=>r.requestedForce,name:'요청(포화 전)',color:'#985279'},{f:r=>r.appliedForce,name:'실제 적용',color:'#208478'}]}];
  bands.forEach((band,j)=>{
   const top=j*110,cy=top+67,height=30,values=all.flatMap(r=>band.series.map(s=>s.f(r))),M=Math.max(j===2?10:.1,...values.map(Math.abs));
   g.strokeStyle='#dae4ec';g.beginPath();g.moveTo(left,cy);g.lineTo(right,cy);g.stroke();g.fillStyle='#465e71';g.font='14px system-ui';g.fillText(band.label+' · ±'+number(M,2),left,top+17);
   band.series.forEach((series,k)=>{g.strokeStyle=series.color;g.fillStyle=series.color;g.fillText(series.name,left+250+k*170,top+17);g.lineWidth=1.5;g.beginPath();run.trace.forEach((r,i)=>{const x=left+r.commandTime/lastTime*(right-left),y=cy-series.f(r)/M*height;i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();});
   if(j===2){g.setLineDash([5,4]);g.strokeStyle='#a55442';for(const bound of [-10,10]){g.beginPath();g.moveTo(left,cy-bound/M*height);g.lineTo(right,cy-bound/M*height);g.stroke();}g.setLineDash([]);}
   const t=run.trace[index]?.commandTime??0,x=left+t/lastTime*(right-left);g.strokeStyle='#25384a';g.beginPath();g.moveTo(x,top+25);g.lineTo(x,top+105);g.stroke();
  });g.fillStyle='#506679';g.font='13px system-ui';g.fillText('0 → '+number(lastTime,2)+' s · 각 R_e 설계에 같은 축 범위 · 세로선: 선택 샘플',left,H-5);
 }
 function renderFrame(){
  const r=chosen();if(!r)return;index=Math.max(0,Math.min(index,r.trace.length-1));const q=r.trace[index];if(!q)return;
  $('calibrationScrub').max=Math.max(0,r.trace.length-1);$('calibrationScrub').value=index;
  text('calibrationClock','기록 재생: t='+number(q.commandTime,2)+' s → '+number(q.nextTime,2)+' s · '+(q.saturated?'포화 발생':'포화 없음')+' · '+r.label);
  text('calibrationEstimate','측정 p='+number(q.measurement[0])+' m / θ='+number(q.measurement[1])+' rad\n추정 p='+number(q.estimate[0])+' m / θ='+number(q.estimate[2])+' rad\n추정 오차 p='+number(q.estimationError[0])+' m / θ='+number(q.estimationError[2])+' rad\ninnovation p='+number(q.innovation?.[0])+' m / θ='+number(q.innovation?.[1])+' rad\nK_e 일부: p←y_p '+number(q.filterGain?.[0]?.[0])+', θ←y_θ '+number(q.filterGain?.[2]?.[1])+'\nNIS='+number(q.nis)+' (진단값, 인증 아님)');
  text('calibrationForce','평가용 분해: 동일 상태를 안다는 가정의 힘 '+number(q.oracleForce)+' N\n추정 오차 기여 합계 '+number(q.requestedForce-q.oracleForce)+' N\n= 포화 전 요청 '+number(q.requestedForce)+' N\n실제 controller에는 truth가 전달되지 않습니다.');
  $('calibrationContributions').innerHTML=q.estimationForceContributions.map((v,i)=>'<tr><td>'+['p','v','θ','ω'][i]+'</td><td>'+number(q.estimationError[i],4)+' '+['m','m/s','rad','rad/s'][i]+'</td><td>'+number(v)+' N</td></tr>').join('');
  text('calibrationOutcome','포화 후 command '+number(q.command)+' N\nactuator 적용 '+number(q.appliedForce)+' N / 외력 '+number(q.external)+' N\n다음 실제 위치 '+number(q.nextTruth[0])+' m / 목표 '+number(q.goal)+' m\n전체 실행 '+r.outcome+' · 목표 기준 '+(r.taskPassed?'충족':'미충족'));
  timeline();
 }
 function render(){
  if(!fit)return;
  text('calibrationParameters','512개 별도 정지 측정 → R_e 대각 성분\n위치: '+fit.Rdiag[0].toExponential(4)+' m²\n각도: '+fit.Rdiag[1].toExponential(4)+' rad²\n표준편차: '+number(fit.standardDeviation[0])+' m / '+number(fit.standardDeviation[1])+' rad\n상수 bias·Q_e·지연은 식별하지 않았습니다.');histogram();
  const tbody=$('calibrationTable').tBodies[0];tbody.replaceChildren();
  for(const r of runs){const tr=tbody.insertRow();tr.dataset.arm=r.armId;const vals=[r.label+' (p '+r.Rdiag[0].toExponential(2)+' m²; θ '+r.Rdiag[1].toExponential(2)+' rad²)',r.outcome+' / 목표 '+(r.taskPassed?'충족':'미충족'),number(r.estimationRmseByState[0])+' m',number(r.positionTrackingRmse_m)+' m',number(r.rmsForce_N)+' N',number(r.rmsCommandSlew_N)+' N',r.saturatedSamples+' / '+r.appliedSteps];vals.forEach(v=>{tr.insertCell().textContent=v;});}
  text('calibrationReplay','동일한 측정·입력 기록으로 관측기만 재생 (보정 R_e 실행에서 수집):\n'+replay.map(r=>m.arms.find(a=>a.id===r.armId).label+' → p 추정 RMSE '+number(r.estimationRmseByState[0])+' m').join('\n')+'\n위 표는 각 controller가 서로 다른 궤적을 만든 폐루프 결과입니다. 이 재생값과 혼동하지 않습니다.');
  const base=runs[0],cal=runs[1],over=runs[2];
  text('calibrationInterpretation','현재 조건의 관찰: 보정 전/후 포화 '+base.saturatedSamples+' → '+cal.saturatedSamples+'회; 추종 RMSE '+number(base.positionTrackingRmse_m)+' → '+number(cal.positionTrackingRmse_m)+' m. '+
   (cal.positionTrackingRmse_m<base.positionTrackingRmse_m?'이 조건에서는 추종도 개선됐습니다.':'이 조건에서는 추종이 개선되지 않았습니다.')+
   ' 100배 R_e의 추정 RMSE는 '+number(over.estimationRmseByState[0])+' m, 추종 RMSE는 '+number(over.positionTrackingRmse_m)+' m입니다. 낮은 추정 오차·낮은 힘 변동만으로 가장 좋은 제어라고 선택하지 않습니다.');
  $('calibrationInspect').hidden=false;renderFrame();
 }
 async function run(id=$('calibrationCase').value){
  const test=m.cases.find(c=>c.id===id);if(!test)throw Error('Unknown frozen calibration case');if(busy)throw Error('Calibration lesson already running');
  pause();busy=true;status='running';lastError=null;caseId=id;$('calibrationCase').value=id;runs=[];index=0;$('calibrationRun').disabled=true;$('calibrationCase').disabled=true;$('calibrationInspect').hidden=true;
  text('calibrationStatus','별도 정지 측정 후 3개 R_e 설계를 실제 MuJoCo에서 비교 중. 위 공통 실험은 설정을 바꾸지 않고 일시정지했습니다.');
  try{
   capture=C.collectStationary(m);fit=C.estimateMeasurementNoise(capture.measurements,m.calibrationScreen);if(!fit.usable)throw Error(fit.warnings.join('; '));
   for(const arm of m.arms){const runner=C.createRun(m,test,arm,fit);let count=0;while(!runner.done){runner.step();if(++count%75===0)await new Promise(resolve=>setTimeout(resolve,0));}const result=runner.result();runs.push(result);if(result.outcome==='execution-error')throw Error(result.reason);}
   replay=C.replayObserver(m,runs[1],m.arms,fit);armId=$('calibrationArm').value;status='completed';
   text('calibrationStatus','실행 완료 · '+caseId+' · 3개 경로의 기록을 비교합니다. 공통 실험의 controller/observer/default는 바꾸지 않았습니다.');render();return getState();
  }catch(e){status='error';lastError=String(e.message);text('calibrationStatus','실험 중단: '+lastError);throw e;}
  finally{busy=false;$('calibrationRun').disabled=false;$('calibrationCase').disabled=false;}
 }
 function seek(i){const r=chosen();if(!r||!Number.isInteger(i)||i<0||i>=r.trace.length)throw Error('Invalid recorded sample');index=i;renderFrame();return getState();}
 function seekFirstSaturation(){const r=chosen(),i=r?.trace.findIndex(q=>q.saturated)??-1;if(i<0){text('calibrationClock','이 실행에는 포화 샘플이 없습니다.');return getState();}return seek(i);}
 $('calibrationRun').onclick=()=>{run().catch(()=>{});};$('calibrationArm').onchange=()=>{armId=$('calibrationArm').value;index=0;renderFrame();};$('calibrationScrub').oninput=()=>seek(Number($('calibrationScrub').value));$('calibrationFirstSaturation').onclick=seekFirstSaturation;
 $('openCalibration').onclick=()=>$('calibrationLesson').scrollIntoView({behavior:'smooth',block:'start'});$('calibrationRun').disabled=false;
 return {run,getState,seek,seekFirstSaturation};
}};
