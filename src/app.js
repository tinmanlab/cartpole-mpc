'use strict';
(async function(){
  const $=function(id){return document.getElementById(id);};
  const L=ControlLab,P=Plant,actor=CONTROL_LAB_ACTOR,residualModel=CONTROL_LAB_RESIDUAL||null;
  const TOPICS=CONTROL_LAB_TOPICS,ORDER=CONTROL_LAB_TOPIC_ORDER;
  const runtimeInputs=[...document.querySelectorAll('button,select')];runtimeInputs.forEach(e=>e.disabled=true);
  const {createMujocoBackend}=await import('./src/mujoco_backend.mjs');
  const physics=await createMujocoBackend();L.setPhysicsBackend(physics);
  $('physicsBadge').textContent='MuJoCo WASM '+physics.version;
  $('assetStatus').textContent='Uniform rod · full length 1.0 m · COM 0.5 m · cart 1 kg / pole 0.1 kg · MJCF '+physics.assetSha256.slice(0,12);
  runtimeInputs.forEach(e=>e.disabled=false);

  const controllerLabels={pid:'PID',lqr:'LQR',linear_mpc:'Linear MPC',hard_mpc:'Constrained MPC · hard rail',scenario_mpc:'Scenario-risk MPC',state_mpc:'MPC + goal-error soft penalty',ltv_mpc:'LTV approximation · one iLQR update',centroidal_mpc:'Reduced CoM planner + LQR',full_nmpc:'Full nonlinear NMPC',supervised_nmpc:'NMPC + heuristic backup',ppo:'PPO'};
  const observerLabels={truth:'Truth',raw:'Raw + diff',kf:'KF',ekf:'EKF',so2:'SO(2) error bridge',ukf:'UKF',mhe:'Nonlinear shooting MHE',residual:'Learned residual',adaptive:'Adaptive R · outlier/reliability bridge'};
  const relationControllers="<p>선택 축이며 성능 승급 순서가 아닙니다.</p><div class='relation'><div class='rel-row'><span class='rel-node'>피드백 설계: PID / LQR / learned policy</span></div><div class='rel-row'><span class='rel-node'>예측 모델: LTI / LTV / nonlinear</span></div><div class='rel-row'><span class='rel-node'>차수: reduced / full</span><span class='rel-node'>제약: input / state / contact</span></div><div class='rel-row'><span class='rel-node'>전사: single / multiple shooting</span><span class='rel-node'>해법: QP / SQP / iLQR / IPM</span></div></div>";
  const relationObservers="<p>서로 다른 문제 설정에 맞는 선택지입니다.</p><div class='relation'><div class='rel-row'><span class='rel-node'>재귀 추정: KF / EKF / sigma-point UKF</span></div><div class='rel-row'><span class='rel-node'>오차 geometry: Euclidean / invariant</span><span class='rel-node'>window: MHE</span></div><div class='rel-row'><span class='rel-node'>학습 위치: contact event / measurement / output residual / covariance</span></div></div>";
  let topic=(location.hash||'#overview').slice(1);if(!TOPICS[topic])topic='overview';
  let runtimeError=null,lastTick=null,activeConfig=null,componentSE=[0,0,0,0],trackingSE=0;
  let plant,controller,observer,running=true,lastTs=0,acc=0,trace=[],sumErr=0,nErr=0,lastEstimate=[0,0,0,0],lastMeasurement=[0,0],lastInnovation=null,lastForce=0,lastAppliedForce=0;

  const supportsTerminal=name=>['linear_mpc','hard_mpc'].includes(name);
  function terminalOptions(name,choice=$('terminalCost').value,explicit=false){
    if(!['original','dare'].includes(choice))throw Error('Unknown terminal cost option');
    if(!supportsTerminal(name)){if(explicit)throw Error('Unsupported terminal cost option for '+name);return {};}
    return {terminalCost:choice};
  }
  function terminalPayload(){return controller?.terminalInfo?{...controller.terminalInfo,matrix:controller.Qf.map(row=>row.slice())}:null;}
  function spec(){return Object.assign({},P.DEFAULT_SPEC,{actuator:'ideal',force:10,friction:0});}
  function nav(){
    let last='';
    $('topicNav').innerHTML=ORDER.map(function(k){
      const q=TOPICS[k],sep=q.group!==last&&last?"<span class='topic-sep'></span>":'';last=q.group;
      const prefix=q.group==='Controller'?'C · ':q.group==='Observer'?'O · ':q.group==='Hybrid estimator'?'H · ':'';
      return sep+"<button data-topic='"+k+"' class='"+(k===topic?'active':'')+"'>"+prefix+q.title.split(' — ')[0]+"</button>";
    }).join('');
    Array.from($('topicNav').querySelectorAll('button')).forEach(function(b){b.addEventListener('click',function(){setTopic(b.dataset.topic);});});
  }
  function relationForCurrent(){
    return TOPICS[topic].group==='Controller'?relationControllers:
      ['Observer','Hybrid estimator'].includes(TOPICS[topic].group)?relationObservers:relationControllers+relationObservers;
  }
  function showLessonScope(){
    const names={implemented:'현재 구현',analogue:'제한된 구조적 비유',paper:'논문 설명 · 재현 아님',concept:'설계·평가 개념'};
    const a=activeConfig;
    $('lessonScope').textContent=(names[TOPICS[topic].scope]||'범위 미지정')+' · '+
      (a?'실행: '+controllerLabels[a.controller]+' / '+observerLabels[a.observer]:'실행 준비 중')+
      ' · 주제 읽기는 실행 모드를 바꾸지 않습니다.';
  }
  function metricPayload(){
    const samples=nErr/4;
    return {samples,stateUnits:['m','m/s','rad','rad/s'],estimationRmseByState:samples?componentSE.map(v=>Math.sqrt(v/samples)):null,
      positionTrackingRmse:samples?Math.sqrt(trackingSE/samples):null,scope:'cumulative since successful reset; tracking and estimation are separate'};
  }
  function renderTopic(){
    const q=TOPICS[topic];
    $('topicKind').textContent=q.group;$('topicTitle').textContent=q.title;$('topicLead').textContent=q.lead;
    $('topicChips').innerHTML=q.chips.map(function(x){return "<span class='chip'>"+x+"</span>";}).join('');
    $('topicBody').innerHTML=q.body+"<div class='section'><h3>근거와 구현 범위</h3>"+q.sources.map(v=>"<p><a class='source-link' href='"+v.url+"' target='_blank' rel='noopener noreferrer'>"+v.label+"</a></p>").join('')+"</div><div class='section'><h3>선택 축</h3>"+relationForCurrent()+"</div>";
    showLessonScope();
    const btn=$('useTopic'),rt=$('topicRuntime');
    if(q.runtime){
      btn.disabled=false;const parts=[];
      if(q.runtime.controller)parts.push(controllerLabels[q.runtime.controller]);
      if(q.runtime.observer)parts.push(observerLabels[q.runtime.observer]);
      if(q.runtime.scenario)parts.push('scenario: '+q.runtime.scenario);
      rt.textContent=parts.join(' / ')||'부분 설정 적용';btn.textContent=q.scope==='analogue'?'비유 모드 적용':'이 설정 적용';
    }else{btn.disabled=true;rt.textContent=q.scope==='implemented'?'offline 구현 · 문서의 명령으로 실행':'설명 전용 · 현재 실행은 유지';}
    nav();updateContextGraphLabels();
  }
  function setTopic(k){if(!TOPICS[k])return;topic=k;history.replaceState(null,'','#'+k);renderTopic();}
  function useTopic(){
    const r=TOPICS[topic].runtime;if(!r)return;
    if(r.controller)$('controller').value=r.controller;if(r.observer)$('observer').value=r.observer;if(r.scenario)$('scenario').value=r.scenario;rebuild();
  }
  function rebuild(){
    try{
      // Construct all pieces before publishing a new live loop. Failure retains the
      // previous valid snapshot but stops it; never pair a new plant with an old controller.
      const nextPlant=new L.LabPlant({seed:31,scenario:$('scenario').value,spec:spec()});nextPlant.goal=+$('goal').value;
      $('terminalCost').disabled=!supportsTerminal($('controller').value);
      const nextController=L.makeController($('controller').value,nextPlant.spec,actor,terminalOptions($('controller').value));
      const nextObserver=L.makeObserver($('observer').value,nextPlant.spec,{model:residualModel,R:nextPlant.measurementVariance()});
      const y=nextPlant.sensor();nextObserver.reset(y,nextPlant.s);nextController.reset();
      const modelGeometry=physics.geometry(nextPlant.s,nextPlant.spec,nextPlant.params);
      plant=nextPlant;controller=nextController;observer=nextObserver;
      activeConfig={controller:$('controller').value,observer:$('observer').value,scenario:$('scenario').value};
      runtimeError=null;lastTick=null;acc=0;lastTs=0;trace=[];sumErr=0;nErr=0;componentSE=[0,0,0,0];trackingSE=0;
      lastEstimate=observer.outputX&&observer.outputX.slice?observer.outputX.slice():(observer.x&&observer.x.slice?observer.x.slice():plant.s.slice());
      lastMeasurement=y;lastInnovation=null;lastForce=0;lastAppliedForce=0;running=true;
      $('assetStatus').textContent='MJCF '+physics.assetSha256.slice(0,12)+' · true rod '+(2*modelGeometry.poleCom[2]).toFixed(3)+' m · mass '+modelGeometry.mass.map(v=>v.toFixed(3)).join(' / ')+' kg · ideal force actuator';
      $('terminalStatus').textContent=controller.terminalInfo?.kind==='dare'?'Riccati: 전체 결합항 유지 · 수렴 확인 · nominal residual '+controller.terminalInfo.normalizedResidual.toExponential(1)+' · 전역 안전 보장 아님':controller.terminalInfo?'Original: 설정된 대각 terminal penalty · 이전 비교 기준 유지':'Terminal 선택은 Linear MPC / hard-rail MPC에만 적용됩니다.';
      updatePipeline();renderAll();return true;
    }catch(error){
      running=false;runtimeError=String(error.message);
      if(!plant||!controller||!observer)throw error;
      $('terminalStatus').textContent='설정 적용 실패 · 이전 상태에서 정지 · Reset 필요';renderAll();return false;
    }
  }
  function stepOne(){
    if(runtimeError)return false;const wallStart=performance.now(),sourceTime=plant.steps*L.DT;
    try{
    // Discrete-time order: x_hat_k -> u_k -> plant x_(k+1) -> y_(k+1)
    // -> observer predict/update -> x_hat_(k+1). This keeps measurement,
    // estimate and truth on the same timestamp.
    const u=controller.act(lastEstimate,plant.goal),applyAt=performance.now();
    if(!Number.isFinite(u))throw Error('Non-finite controller output rejected');
    const out=plant.step(u),y=plant.sensor(),xh=observer.step(plant.sensorMeta.fresh?y:null,u,out.state);
    const completedAt=performance.now();lastTick={sourceStateTime:sourceTime,postStateTime:plant.steps*L.DT,solveToApplyMs:applyAt-wallStart,computeMs:completedAt-wallStart,deadlineMs:L.DT*1000,missedComputeDeadline:completedAt-wallStart>L.DT*1000,scope:'synchronous simulated loop; not hardware timing'};
    const e=out.state.map(function(v,i){return i===2?L.wrap(v-xh[i]):v-xh[i];});
    e.forEach((v,i)=>componentSE[i]+=v*v);trackingSE+=(out.state[0]-plant.goal)**2;
    sumErr+=e.reduce(function(ss,v){return ss+v*v;},0);nErr+=4;lastEstimate=xh.slice();lastMeasurement=y.slice();
    lastInnovation=observer.last?.innovation?.slice()??null;lastForce=u;lastAppliedForce=out.appliedCommand??u;
    const Pout=observer.outputCovariance?observer.outputCovariance():observer.P;
    trace.push({t:plant.steps*L.DT,terminalCost:controller.terminalInfo?.kind??null,timing:lastTick,truth:out.state.slice(),estimate:xh.slice(),u:u,external:out.external,innovation:lastInnovation?.slice()??null,
      P:Pout?Pout.map(function(r){return r.slice();}):null,baseP:observer.P?observer.P.map(function(r){return r.slice();}):null,
      measurementUsed:observer.last?.measurementUsed??plant.sensorMeta.fresh,sensorFresh:plant.sensorMeta.fresh,sensorFault:plant.sensorMeta.fault,observerCost:observer.last?.cost??null,observerIterations:observer.last?.iterations??0,observerWindow:observer.last?.window??0,
      reliability:observer.last&&observer.last.reliability?observer.last.reliability.slice():null,appliedCommand:out.appliedCommand??u,solveMs:controller.lastSolveMs||0,iterations:controller.lastIterations||0});
    if(trace.length>500)trace.shift();if(out.failed){running=false;runtimeError='Simulation state envelope exceeded';return false;}return true;
    }catch(error){runtimeError=error.message;running=false;return false;}
  }
  function drawCart(g,s,color,dashed,alpha,scaleX,ground,center){
    // Draw the actual compiled MJCF geometry using MuJoCo forward kinematics.
    // No decorative wheels or separately hard-coded rod/cart dimensions.
    const a=physics.geometry(s,plant.spec,s===plant.s?plant.params:null),xy=q=>[center+q[0]*scaleX,ground-q[2]*scaleX],c=xy(a.cart),p=xy(a.pivot),t=xy(a.tip);
    g.save();g.globalAlpha=alpha;g.strokeStyle=color;g.fillStyle=color;g.setLineDash(dashed?[7,5]:[]);g.lineWidth=2;
    g.strokeRect(c[0]-a.cartHalfSize[0]*scaleX,c[1]-a.cartHalfSize[2]*scaleX,2*a.cartHalfSize[0]*scaleX,2*a.cartHalfSize[2]*scaleX);
    g.lineWidth=Math.max(2,2*a.rodRadius*scaleX);g.lineCap='round';g.beginPath();g.moveTo(...p);g.lineTo(...t);g.stroke();g.beginPath();g.arc(p[0],p[1],4,0,2*Math.PI);g.fill();g.restore();
  }
  function world(){
    const c=$('world'),g=c.getContext('2d'),W=c.width,H=c.height,ground=310,center=W/2,scaleX=170;g.clearRect(0,0,W,H);g.fillStyle='#fbfdff';g.fillRect(0,0,W,H);const asset=physics.geometry(plant.s,plant.spec,plant.params);g.fillStyle='#d9e2e9';g.fillRect(center+(asset.rail[0]-asset.railHalfSize[0])*scaleX,ground-(asset.rail[2]+asset.railHalfSize[2])*scaleX,2*asset.railHalfSize[0]*scaleX,2*asset.railHalfSize[2]*scaleX);
    for(let m=-2;m<=2;m++){const xx=center+m*scaleX;g.strokeStyle='#edf1f5';g.beginPath();g.moveTo(xx,ground-7);g.lineTo(xx,ground+7);g.stroke();g.fillStyle='#8192a1';g.font='11px system-ui';g.fillText(m+'m',xx-8,ground+21);}
    c.dataset.ghosts=controller.predictionSpace==='reduced-com'?'none-reduced-plan':controller.lastBackup?'primary-plan-not-applied':'nominal-plan';
    const pred=controller&&controller.lastPrediction;if(Array.isArray(pred)&&pred.length>2){const stride=Math.max(4,Math.floor(pred.length/6));for(let k=4;k<pred.length;k+=stride)drawCart(g,pred[k],'#14845d',true,.13,scaleX,ground,center);}
    drawCart(g,plant.s,'#2f78c4',false,.92,scaleX,ground,center);drawCart(g,lastEstimate,'#b87825',true,.9,scaleX,ground,center);
    const gx=center+plant.goal*scaleX;g.strokeStyle='#14845d';g.setLineDash([4,4]);g.beginPath();g.moveTo(gx,ground-56);g.lineTo(gx,ground+2);g.stroke();g.setLineDash([]);
    // Reduced c_dot has no height coordinate: show it in the labelled context plot only.
  }
  function drawSeries(canvas,series,xDomain=[trace[0]?.t??0,trace.at(-1)?.t??L.DT],xTitle='simulation t [s]'){
    // One labelled scale per physical quantity. Missing samples break the line.
    const g=canvas.getContext('2d'),W=canvas.width,H=canvas.height;g.clearRect(0,0,W,H);
    canvas.dataset.series=JSON.stringify(series.map(q=>({label:q.label,unit:q.unit})));canvas.setAttribute('aria-label',series.map(q=>q.label+' ['+q.unit+']').join('; ')+'; '+xTitle);
    const band=(H-19)/Math.max(1,series.length),left=7,right=W-7,dx=Math.max(1e-9,xDomain[1]-xDomain[0]);
    series.forEach((q,row)=>{
      const vals=q.values.filter(Number.isFinite),M=Math.max(.00001,...vals.map(Math.abs)),top=row*band,cy=top+band*.64,amp=band*.23;
      g.setLineDash([]);g.fillStyle=q.color;g.font='10px system-ui';g.fillText(q.label+' ['+q.unit+'] · '+(vals.length?'±'+M.toPrecision(2):'unavailable'),left,top+11);
      g.strokeStyle='#dce5ed';g.beginPath();g.moveTo(left,cy);g.lineTo(right,cy);g.stroke();
      g.strokeStyle=q.color;g.lineWidth=1.7;g.setLineDash(q.dash||[]);g.beginPath();let connected=false;
      q.values.forEach((v,i)=>{if(!Number.isFinite(v)){connected=false;return;}const time=q.times?.[i]??(xDomain[0]+i/Math.max(1,q.values.length-1)*dx);const xx=left+(time-xDomain[0])/dx*(right-left),yy=cy-v/M*amp;if(connected)g.lineTo(xx,yy);else g.moveTo(xx,yy);connected=true;});g.stroke();
    });
    g.setLineDash([]);g.fillStyle='#667b8e';g.font='9px system-ui';g.fillText(xDomain[0].toFixed(2)+' → '+xDomain[1].toFixed(2)+' · '+xTitle,left,H-3);
  }
  function stateChart(){drawSeries($('stateChart'),[{label:'θ truth',unit:'deg',values:trace.map(q=>q.truth[2]*180/Math.PI),color:'#2f78c4'},{label:'θ estimate',unit:'deg',values:trace.map(q=>q.estimate[2]*180/Math.PI),color:'#b87825',dash:[5,4]}]);}
  function controlChart(){drawSeries($('controlChart'),[{label:'command',unit:'N',values:trace.map(q=>q.u),color:'#14845d'},{label:'applied actuator',unit:'N',values:trace.map(q=>q.appliedCommand??q.u),color:'#b87825',dash:[5,3]},{label:'external push',unit:'N',values:trace.map(q=>q.external),color:'#bd4b4b',dash:[2,3]}]);}
  function updateContextGraphLabels(){
    $('contextGraphTitle').textContent='현재 실행의 진단';
    $('contextGraphSub').textContent='단위별 독립 축 · 아래 라벨 확인';
  }
  function contextChart(){
    const canvas=$('contextChart'),a=activeConfig;
    if(!a)return;
    if(TOPICS[topic].group==='Controller'){
      if(controller.predictionSpace==='reduced-com'){
        canvas.dataset.space='reduced-com';const z=controller.lastPredictionReduced||[],u=controller.lastControls||[];
        drawSeries(canvas,[{label:'planned CoM c',unit:'m',values:z.map(q=>q[0]),color:'#14845d',times:z.map((_,k)=>k*L.DT)},{label:'planned c_dot',unit:'m/s',values:z.map(q=>q[1]),color:'#278b98',times:z.map((_,k)=>k*L.DT)}],[0,controller.N*L.DT],'nominal horizon [s]');return;
      }
      const pred=controller.lastPrediction||[],u=controller.lastControls||[];
      if(pred.length){canvas.dataset.space='nominal-full-state';drawSeries(canvas,[{label:'planned θ',unit:'deg',values:pred.map(q=>q[2]*180/Math.PI),color:'#14845d',times:pred.map((_,k)=>k*L.DT)},{label:'planned force',unit:'N',values:u,color:'#7659b0',times:u.map((_,k)=>k*L.DT)}],[0,(pred.length-1)*L.DT],controller.lastBackup?'PRIMARY only; backup differs [s]':'nominal horizon [s]');return;}
      canvas.dataset.space='no-horizon';const g=canvas.getContext('2d');g.clearRect(0,0,canvas.width,canvas.height);g.fillStyle='#667b8e';g.font='11px system-ui';g.fillText('현재 '+controllerLabels[a.controller],7,22);g.fillText('이 controller는 horizon을 출력하지 않습니다.',7,44);return;
    }
    if(a.observer==='mhe'){
      canvas.dataset.space='window-fit';drawSeries(canvas,[{label:'MHE weighted fit cost',unit:'dimensionless',values:trace.map(q=>q.observerCost),color:'#7659b0'},{label:'iterations',unit:'count',values:trace.map(q=>q.observerIterations),color:'#278b98'}]);return;
    }
    if(trace.some(q=>q.P)&&a.observer!=='truth'){
      canvas.dataset.space='estimator-covariance';const series=[{label:'model σ(v)',unit:'m/s',values:trace.map(q=>q.P?Math.sqrt(Math.max(0,q.P[1][1])):null),color:'#7659b0'},{label:'model σ(ω)',unit:'rad/s',values:trace.map(q=>q.P?Math.sqrt(Math.max(0,q.P[3][3])):null),color:'#278b98'}];
      if(trace.some(q=>q.reliability))series.push({label:'heuristic reliability',unit:'0–1',values:trace.map(q=>q.reliability?Math.min(...q.reliability):null),color:'#14845d'});
      drawSeries(canvas,series);return;
    }
    canvas.dataset.space='unavailable';canvas.dataset.series='[]';const g=canvas.getContext('2d');g.clearRect(0,0,canvas.width,canvas.height);g.fillStyle='#667b8e';g.font='11px system-ui';g.fillText(a.observer==='truth'?'Oracle: 정답 직접 사용 · 추정 신뢰구간 아님':'현재 output의 calibrated covariance 없음',7,25);
  }
  function renderMetrics(){
    $('clock').textContent='t='+(plant.steps*L.DT).toFixed(2)+' s';$('mTheta').textContent=(plant.s[2]*180/Math.PI).toFixed(2)+'°';$('mThetaHat').textContent=(lastEstimate[2]*180/Math.PI).toFixed(2)+'°';const metrics=metricPayload();$('mRmse').textContent=metrics.estimationRmseByState?metrics.estimationRmseByState[0].toFixed(3)+' m':'—';$('mTracking').textContent=metrics.positionTrackingRmse===null?'—':metrics.positionTrackingRmse.toFixed(3)+' m';$('mForce').textContent=lastForce.toFixed(2)+' / '+lastAppliedForce.toFixed(2)+' N';$('mInnov').textContent=lastInnovation?lastInnovation[0].toFixed(2)+' m / '+lastInnovation[1].toFixed(2)+' rad':'—';$('mInnov').title=activeConfig?.observer==='mhe'?'post-fit window residual; not prefit NIS innovation':'position/angle prefit innovation; never a mixed-unit norm';
    $('mSolve').textContent=controller&&Number.isFinite(controller.lastSolveMs)&&controller.lastSolveMs>0?controller.lastSolveMs.toFixed(2)+' ms':'—';const horizon=(controller?.lastPredictionReduced?.length||controller?.lastPrediction?.length||1)-1;$('mIter').textContent=horizon?String(horizon)+' / '+String(controller.lastIterations??0):'—';$('statusBadge').textContent=runtimeError?'FAULT':running?'RUNNING':'PAUSED';if(runtimeError)$('assetStatus').textContent=runtimeError+' · reset required; no hardware command is issued';$('play').textContent=running?'일시정지':'재생';
  }
  function updatePipeline(){if(!activeConfig)return;$('pipeSensor').textContent='y=['+lastMeasurement.map(function(v){return v.toFixed(2);}).join(',')+']';$('pipeObserver').textContent=observerLabels[activeConfig.observer];$('pipeController').textContent=controllerLabels[activeConfig.controller];$('informationScope').textContent=activeConfig.observer==='truth'?'ORACLE: controller도 simulator truth를 받습니다 · 배포 가능한 센서가 아닙니다.':'Controller 입력은 observer 출력입니다 · covariance/학습의 simulation 정보 사용은 별도 표기합니다.';}
  function renderAll(){world();stateChart();controlChart();contextChart();renderMetrics();updatePipeline();showLessonScope();}
  function loop(ts){if(!lastTs)lastTs=ts;const dt=Math.min(.08,(ts-lastTs)/1000);lastTs=ts;if(running){acc+=dt;let n=0;while(running&&acc>=L.DT&&n<4){stepOne();acc-=L.DT;n++;}}renderAll();requestAnimationFrame(loop);}
  async function compare(){
    // Freeze this experiment's configuration; yield between cells so the UI remains responsive.
    const sc=$('compareScenario').value,goal=+$('goal').value,terminalChoice=$('terminalCost').value;
    const cs=['pid','lqr','linear_mpc','hard_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','supervised_nmpc','ppo'];
    const os=['truth','raw','kf','ekf','so2','residual','adaptive'];
    running=false;$('compare').disabled=true;
    $('matrix').innerHTML='<thead><tr><th>Controller</th>'+os.map(o=>'<th>'+observerLabels[o]+'</th>').join('')+'</tr></thead><tbody></tbody>';
    try{
      for(const c of cs){
        const row=$('matrix').tBodies[0].insertRow();row.dataset.terminalCost=supportsTerminal(c)?terminalChoice:'not-applicable';row.insertCell().textContent=controllerLabels[c]+(supportsTerminal(c)?' · '+(terminalChoice==='dare'?'Riccati':'Original'):'');
        for(const o of os){
          const cell=row.insertCell();
          try{
            const r=L.runEpisode({controller:c,observer:o,scenario:sc,seed:77,steps:240,goal,actor,residualModel,controllerOpts:terminalOptions(c,terminalChoice),pushAt:96,pushForce:3});
            cell.className=r.failed?'fail':'pass';
            const tracking=Number.isFinite(r.positionTrackingRmse)?r.positionTrackingRmse.toFixed(2):'—',est=Number.isFinite(r.estimationRmseByState?.[0])?r.estimationRmseByState[0].toFixed(2):'—';
            cell.textContent=(r.failed?'FAIL':'DONE')+' · track '+tracking+' m / est-p '+est+' m';cell.title='DONE means requested steps completed within the tested envelope; not target achievement, solver optimality or safety. Timing is host-specific.';
          }catch(error){
            cell.className='fail';cell.textContent=String(error.message).startsWith('QP ')?'REJECT':'ERROR';
            cell.title=String(error.message); // DOM text, never unescaped numerical-error HTML.
          }
          await new Promise(resolve=>setTimeout(resolve,0));
        }
      }
    }finally{$('compare').disabled=false;renderMetrics();}
  }
  function statePayload(){
    const Pout=observer.outputCovariance?observer.outputCovariance():observer.P;
    return {physics:L.physicsInfo(),terminalCost:terminalPayload(),fault:runtimeError,timing:lastTick,topic:topic,controller:activeConfig.controller,observer:activeConfig.observer,scenario:activeConfig.scenario,t:plant.steps*L.DT,goal:plant.goal,metrics:metricPayload(),
      truth:plant.s.slice(),estimate:lastEstimate.slice(),measurement:lastMeasurement.slice(),innovation:lastInnovation?.slice()??null,force:lastForce,appliedForce:lastAppliedForce,
      P:Pout?Pout.map(function(r){return r.slice();}):null,baseP:observer.P?observer.P.map(function(r){return r.slice();}):null,
      covarianceScope:Pout?'output-estimate':'base-filter-only-or-not-applicable',
      sensorFresh:plant.sensorMeta.fresh,measurementUsed:observer.last?.measurementUsed??plant.sensorMeta.fresh,measurementCovarianceSource:'injected-noise-oracle',
      observerDiagnostics:{iterations:observer.last?.iterations??0,cost:observer.last?.cost??null,window:observer.last?.window??0},
      solver:{predictionSpace:controller.predictionSpace??'nominal-full-state',predictionAppliesTo:controller.lastBackup?'primary-not-applied':'nominal-plan',updateAccepted:controller.lastUpdateAccepted??null,termination:controller.lastTermination??null,implementation:controller.lastSolver??controller.name,kktResidual:controller.lastKktResidual??null,primalResidual:controller.lastPrimalResidual??null,stateConstrained:controller.lastStateConstrained??false,converged:controller.lastConverged??null,solveMs:controller.lastSolveMs||0,iterations:controller.lastIterations||0,horizon:(controller.lastPredictionReduced?.length||controller.lastPrediction?.length||1)-1,cost:controller.lastCost===undefined?null:controller.lastCost},
      rmse:Math.sqrt(sumErr/Math.max(1,nErr)),legacyRmseScope:'mixed-units; not a controller score',running:running};
  }
  async function registerWebMCP(){
    const mc=document.modelContext||navigator.modelContext;if(!mc||!mc.registerTool){$('webmcpBadge').textContent='WebMCP unavailable · UI 정상';$('webmcpBadge').classList.add('warn');return;}const ac=new AbortController();window.__controlLabV2Abort=ac;const ro={readOnlyHint:true,untrustedContentHint:false},rw={readOnlyHint:false,untrustedContentHint:false};
    const reg=async function(tool){return mc.registerTool(tool,{signal:ac.signal});};
    try{
      await reg({name:'cartpole_get_state',description:'Read the detailed CartPole control-observer lab state and solver metrics.',inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:ro,execute:async function(){return JSON.stringify(statePayload());}});
      await reg({name:'cartpole_set_controller',description:'Select a controller.',inputSchema:{type:'object',properties:{controller:{type:'string',enum:['pid','lqr','linear_mpc','hard_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','supervised_nmpc','ppo']},terminalCost:{type:'string',enum:['original','dare']}},required:['controller'],additionalProperties:false},annotations:rw,execute:async function(a){terminalOptions(a.controller,a.terminalCost??$('terminalCost').value,a.terminalCost!==undefined);$('controller').value=a.controller;if(a.terminalCost!==undefined)$('terminalCost').value=a.terminalCost;if(!rebuild())throw Error(runtimeError);return 'controller='+a.controller;}});
      await reg({name:'cartpole_set_observer',description:'Select an observer.',inputSchema:{type:'object',properties:{observer:{type:'string',enum:['truth','raw','kf','ekf','ukf','so2','mhe','residual','adaptive']}},required:['observer'],additionalProperties:false},annotations:rw,execute:async function(a){$('observer').value=a.observer;rebuild();return 'observer='+a.observer;}});
      await reg({name:'cartpole_set_topic',description:'Open an educational topic without changing plant state.',inputSchema:{type:'object',properties:{topic:{type:'string',enum:ORDER}},required:['topic'],additionalProperties:false},annotations:rw,execute:async function(a){setTopic(a.topic);return 'topic='+a.topic;}});
      await reg({name:'cartpole_set_scenario',description:'Select scenario.',inputSchema:{type:'object',properties:{scenario:{type:'string',enum:['nominal','sensor','model','mixed','bias','actuator','latency','colored','dropout','stuck','jitter','torque_speed','thermal','nonlinear','sim2real','glitch']}},required:['scenario'],additionalProperties:false},annotations:rw,execute:async function(a){$('scenario').value=a.scenario;rebuild();return 'scenario='+a.scenario;}});
      await reg({name:'cartpole_apply_push',description:'Apply bounded external force.',inputSchema:{type:'object',properties:{force:{type:'number',minimum:-8,maximum:8},steps:{type:'integer',minimum:1,maximum:50}},required:['force'],additionalProperties:false},annotations:rw,execute:async function(a){plant.applyPush(a.force,a.steps||10);return 'push='+a.force;}});
      await reg({name:'cartpole_reset',description:'Reset experiment.',inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:rw,execute:async function(){rebuild();return 'reset';}});
      await reg({name:'cartpole_run_steps',description:'Advance live experiment.',inputSchema:{type:'object',properties:{steps:{type:'integer',minimum:1,maximum:500}},required:['steps'],additionalProperties:false},annotations:rw,execute:async function(a){for(let i=0;i<a.steps;i++)if(!stepOne())break;renderAll();return JSON.stringify(statePayload());}});
      await reg({name:'cartpole_run_probe',description:'Run deterministic offline controller-observer probe without changing live state.',inputSchema:{type:'object',properties:{controller:{type:'string',enum:['pid','lqr','linear_mpc','hard_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','supervised_nmpc','ppo']},observer:{type:'string',enum:['truth','raw','kf','ekf','ukf','so2','mhe','residual','adaptive']},scenario:{type:'string',enum:['nominal','sensor','model','mixed','bias','actuator','latency','colored','dropout','stuck','jitter','torque_speed','thermal','nonlinear','sim2real','glitch']},terminalCost:{type:'string',enum:['original','dare']},seed:{type:'integer',minimum:1,maximum:1000000},steps:{type:'integer',minimum:20,maximum:500},pushForce:{type:'number',minimum:-8,maximum:8}},required:['controller','observer','scenario'],additionalProperties:false},annotations:ro,execute:async function(a){return JSON.stringify(L.runEpisode({controller:a.controller,controllerOpts:terminalOptions(a.controller,a.terminalCost??$('terminalCost').value,a.terminalCost!==undefined),observer:a.observer,scenario:a.scenario,seed:a.seed||77,steps:a.steps||300,goal:+$('goal').value,actor:actor,residualModel:residualModel,pushAt:Math.min(120,Math.floor((a.steps||300)*.4)),pushForce:a.pushForce===undefined?(a.scenario==='nonlinear'?0:3):a.pushForce}));}});
      $('webmcpBadge').textContent='WebMCP tools registered';$('webmcpBadge').classList.add('ok');
    }catch(e){console.warn(e);$('webmcpBadge').textContent='WebMCP registration failed';$('webmcpBadge').classList.add('warn');}
  }
  $('useTopic').addEventListener('click',useTopic);['controller','observer','scenario','goal','terminalCost'].forEach(function(id){$(id).addEventListener('change',rebuild);});$('play').addEventListener('click',function(){if(runtimeError)return;running=!running;renderMetrics();});$('step').addEventListener('click',function(){running=false;stepOne();renderAll();});$('reset').addEventListener('click',rebuild);$('pushL').addEventListener('click',function(){plant.applyPush(-3,10);});$('pushR').addEventListener('click',function(){plant.applyPush(3,10);});$('compare').addEventListener('click',compare);
  window.addEventListener('hashchange',function(){const k=(location.hash||'#overview').slice(1);if(TOPICS[k]){topic=k;renderTopic();}});
  $('controller').value='hard_mpc';$('terminalCost').value='original';$('observer').value='ekf';$('scenario').value='nominal';$('goal').value='0';renderTopic();rebuild();await registerWebMCP();window.controlLab={getState:statePayload,getTrace:()=>trace.slice(),pause:()=>{running=false;},step:n=>{for(let i=0;i<n;i++)if(!stepOne())break;renderAll();return statePayload();}};window.calibrationLesson=CalibrationLessonView.mount({manifest:CONTROL_LAB_CALIBRATION_MANIFEST,processManifest:CONTROL_LAB_PROCESS_MANIFEST,pause:()=>{running=false;renderAll();}});window.__labReady=true;requestAnimationFrame(loop);
})().catch(error=>{console.error(error);document.getElementById('statusBadge').textContent='LOAD ERROR';document.getElementById('assetStatus').textContent=error.message;document.querySelectorAll('button,select').forEach(e=>e.disabled=true);window.__labLoadError=error.message;});