'use strict';
(function(){
  const $=function(id){return document.getElementById(id);};
  const L=ControlLab,P=Plant,actor=CONTROL_LAB_ACTOR,residualModel=CONTROL_LAB_RESIDUAL||null;
  const TOPICS=CONTROL_LAB_TOPICS,ORDER=CONTROL_LAB_TOPIC_ORDER;
  const controllerLabels={pid:'PID',lqr:'LQR',linear_mpc:'Linear MPC',scenario_mpc:'Scenario-risk MPC',state_mpc:'Linear MPC + state bound',ltv_mpc:'LTV MPC · RTI',centroidal_mpc:'Centroidal-style MPC',full_nmpc:'Full nonlinear NMPC',ppo:'PPO'};
  const observerLabels={truth:'Truth',raw:'Raw + diff',kf:'KF',ekf:'EKF',so2:'SO(2) error bridge',mhe:'Nonlinear shooting MHE',residual:'Learned residual',adaptive:'Adaptive R · outlier/reliability bridge'};
  const relationControllers="<div class='relation'><div class='rel-row'><span class='rel-node'>PID · direct feedback</span><span class='rel-arrow'>→ model</span><span class='rel-node'>LQR · Riccati feedback</span><span class='rel-arrow'>→ horizon</span><span class='rel-node'>Linear MPC</span><span class='rel-arrow'>→ state/path constraints</span><span class='rel-node'>Constraint-aware MPC</span><span class='rel-arrow'>→ relinearize</span><span class='rel-node'>LTV / SQP-RTI</span></div><div class='rel-row'><span class='rel-node'>reduced model</span><span class='rel-arrow'>→</span><span class='rel-node'>Centroidal-style MPC</span><span class='rel-arrow'>vs</span><span class='rel-node'>Full nonlinear NMPC</span></div><div class='rel-row'><span class='rel-node'>offline learning</span><span class='rel-arrow'>→</span><span class='rel-node'>PPO</span></div></div>";
  const relationObservers="<div class='relation'><div class='rel-row'><span class='rel-node'>raw</span><span class='rel-arrow'>→ model + uncertainty</span><span class='rel-node'>KF</span><span class='rel-arrow'>→ nonlinear Euclidean</span><span class='rel-node'>EKF</span><span class='rel-arrow'>→ window</span><span class='rel-node'>MHE</span></div><div class='rel-row'><span class='rel-node'>Lie-group symmetry</span><span class='rel-arrow'>→ invariant-error branch</span><span class='rel-node'>InEKF</span></div><div class='rel-row'><span class='rel-node'>Lin · contact event</span><span class='rel-node'>Youm · measurement</span><span class='rel-node'>InNKF · output residual</span><span class='rel-node'>CoCo · process covariance</span><span class='rel-node'>FOCUS · observation reliability</span></div></div>";
  let topic=(location.hash||'#overview').slice(1);if(!TOPICS[topic])topic='overview';
  let plant,controller,observer,running=true,lastTs=0,acc=0,trace=[],sumErr=0,nErr=0,lastEstimate=[0,0,0,0],lastMeasurement=[0,0],lastInnovation=[0,0],lastForce=0,lastAppliedForce=0;

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
    if(['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','ppo'].indexOf(topic)>=0)return relationControllers;
    if(['raw','kf','ekf','inekf','mhe','lin','youm','innkf','coco','focus'].indexOf(topic)>=0)return relationObservers;
    return relationControllers+relationObservers;
  }
  function renderTopic(){
    const q=TOPICS[topic];
    $('topicKind').textContent=q.group;$('topicTitle').textContent=q.title;$('topicLead').textContent=q.lead;
    $('topicChips').innerHTML=q.chips.map(function(x){return "<span class='chip'>"+x+"</span>";}).join('');
    $('topicBody').innerHTML=q.body+"<div class='section'><h3>관계 지도</h3>"+relationForCurrent()+"</div>";
    const btn=$('useTopic'),rt=$('topicRuntime');
    if(q.runtime){
      btn.disabled=false;const parts=[];
      if(q.runtime.controller)parts.push(controllerLabels[q.runtime.controller]);
      if(q.runtime.observer)parts.push(observerLabels[q.runtime.observer]);
      rt.textContent=parts.join(' / ')||'runtime';
    }else{btn.disabled=true;rt.textContent='논문/개념 페이지';}
    nav();updateContextGraphLabels();
  }
  function setTopic(k){if(!TOPICS[k])return;topic=k;history.replaceState(null,'','#'+k);renderTopic();}
  function useTopic(){
    const r=TOPICS[topic].runtime;if(!r)return;
    if(r.controller)$('controller').value=r.controller;if(r.observer)$('observer').value=r.observer;if(r.scenario)$('scenario').value=r.scenario;rebuild();
  }
  function rebuild(){
    plant=new L.LabPlant({seed:31,scenario:$('scenario').value,spec:spec()});plant.goal=+$('goal').value;
    controller=L.makeController($('controller').value,plant.spec,actor);
    observer=L.makeObserver($('observer').value,plant.spec,{model:residualModel,R:plant.measurementVariance()});
    const y=plant.sensor();observer.reset(y,plant.s);controller.reset();trace=[];sumErr=0;nErr=0;
    lastEstimate=observer.outputX&&observer.outputX.slice?observer.outputX.slice():(observer.x&&observer.x.slice?observer.x.slice():plant.s.slice());
    lastMeasurement=y;lastInnovation=[0,0];lastForce=0;lastAppliedForce=0;running=true;updatePipeline();renderAll();
  }
  function stepOne(){
    // Discrete-time order: x_hat_k -> u_k -> plant x_(k+1) -> y_(k+1)
    // -> observer predict/update -> x_hat_(k+1). This keeps measurement,
    // estimate and truth on the same timestamp.
    const u=controller.act(lastEstimate,plant.goal),out=plant.step(u),y=plant.sensor(),xh=observer.step(y,u,out.state);
    const e=out.state.map(function(v,i){return i===2?L.wrap(v-xh[i]):v-xh[i];});
    sumErr+=e.reduce(function(ss,v){return ss+v*v;},0);nErr+=4;lastEstimate=xh.slice();lastMeasurement=y.slice();
    lastInnovation=observer.last&&observer.last.innovation?observer.last.innovation.slice():[0,0];lastForce=u;lastAppliedForce=out.appliedCommand??u;
    const Pout=observer.outputCovariance?observer.outputCovariance():observer.P;
    trace.push({t:plant.steps*L.DT,truth:out.state.slice(),estimate:xh.slice(),u:u,external:out.external,innovation:lastInnovation.slice(),
      P:Pout?Pout.map(function(r){return r.slice();}):null,baseP:observer.P?observer.P.map(function(r){return r.slice();}):null,
      reliability:observer.last&&observer.last.reliability?observer.last.reliability.slice():null,appliedCommand:out.appliedCommand??u,solveMs:controller.lastSolveMs||0,iterations:controller.lastIterations||0});
    if(trace.length>500)trace.shift();if(out.failed)running=false;
  }
  function drawCart(g,s,color,dashed,alpha,scaleX,ground,center){
    const x=center+s[0]*scaleX,theta=s[2],cartY=ground-31,Lp=142;g.save();g.globalAlpha=alpha;g.strokeStyle=color;g.fillStyle=color;g.lineWidth=3;g.setLineDash(dashed?[8,6]:[]);g.strokeRect(x-40,cartY-16,80,26);g.beginPath();g.arc(x-26,ground-9,10,0,Math.PI*2);g.arc(x+26,ground-9,10,0,Math.PI*2);g.stroke();const px=x,py=cartY-16,tx=px+Math.sin(theta)*Lp,ty=py-Math.cos(theta)*Lp;g.beginPath();g.moveTo(px,py);g.lineTo(tx,ty);g.stroke();g.beginPath();g.arc(tx,ty,6,0,Math.PI*2);g.fill();g.restore();
  }
  function world(){
    const c=$('world'),g=c.getContext('2d'),W=c.width,H=c.height,ground=278,center=W/2,scaleX=125;g.clearRect(0,0,W,H);g.fillStyle='#fbfdff';g.fillRect(0,0,W,H);g.strokeStyle='#d9e2e9';g.lineWidth=2;g.beginPath();g.moveTo(18,ground);g.lineTo(W-18,ground);g.stroke();
    for(let m=-3;m<=3;m++){const xx=center+m*scaleX;g.strokeStyle='#edf1f5';g.beginPath();g.moveTo(xx,ground-7);g.lineTo(xx,ground+7);g.stroke();g.fillStyle='#8192a1';g.font='11px system-ui';g.fillText(m+'m',xx-8,ground+21);}
    const pred=controller&&controller.lastPrediction;if(Array.isArray(pred)&&pred.length>2){const stride=Math.max(4,Math.floor(pred.length/6));for(let k=4;k<pred.length;k+=stride)drawCart(g,pred[k],'#14845d',true,.13,scaleX,ground,center);}
    drawCart(g,plant.s,'#2f78c4',false,.92,scaleX,ground,center);drawCart(g,lastEstimate,'#b87825',true,.9,scaleX,ground,center);
    const gx=center+plant.goal*scaleX;g.strokeStyle='#14845d';g.setLineDash([4,4]);g.beginPath();g.moveTo(gx,ground-56);g.lineTo(gx,ground+2);g.stroke();g.setLineDash([]);
    if(controller&&Array.isArray(controller.lastPredictionReduced)&&controller.lastPredictionReduced.length){g.strokeStyle='#14845d';g.lineWidth=2;g.beginPath();controller.lastPredictionReduced.forEach(function(z,i){const xx=center+z[0]*scaleX,yy=ground-4-Math.min(35,Math.abs(z[1])*12);if(i)g.lineTo(xx,yy);else g.moveTo(xx,yy);});g.stroke();}
  }
  function drawSeries(canvas,series){
    const g=canvas.getContext('2d'),W=canvas.width,H=canvas.height;g.clearRect(0,0,W,H);g.strokeStyle='#e7edf2';g.beginPath();g.moveTo(0,H/2);g.lineTo(W,H/2);g.stroke();
    const vals=[];series.forEach(function(s){s.values.forEach(function(v){if(Number.isFinite(v))vals.push(v);});});const M=Math.max.apply(null,[.02].concat(vals.map(Math.abs)));
    series.forEach(function(s){g.strokeStyle=s.color;g.lineWidth=2;g.setLineDash(s.dash||[]);g.beginPath();s.values.forEach(function(v,i){const xx=i/Math.max(1,s.values.length-1)*(W-5)+2,yy=H/2-v/M*(H*.40);if(i)g.lineTo(xx,yy);else g.moveTo(xx,yy);});g.stroke();});g.setLineDash([]);
  }
  function stateChart(){drawSeries($('stateChart'),[{values:trace.map(function(q){return q.truth[2]*180/Math.PI;}),color:'#2f78c4'},{values:trace.map(function(q){return q.estimate[2]*180/Math.PI;}),color:'#b87825',dash:[5,4]}]);}
  function controlChart(){drawSeries($('controlChart'),[{values:trace.map(function(q){return q.u;}),color:'#14845d'},{values:trace.map(function(q){return q.appliedCommand??q.u;}),color:'#b87825',dash:[5,3]},{values:trace.map(function(q){return q.external;}),color:'#bd4b4b',dash:[2,3]}]);}
  function updateContextGraphLabels(){
    const ctl=['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','ppo'].indexOf(topic)>=0;
    $('contextGraphTitle').textContent=ctl?'Controller context':topic==='mhe'?'MHE window':'Observer context';
    $('contextGraphSub').textContent=ctl?'planned horizon / control':topic==='mhe'?'objective / iterations':'sigma(v), sigma(omega), reliability';
  }
  function contextChart(){
    const canvas=$('contextChart'),ctlTopic=['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','ppo'].indexOf(topic)>=0;
    if(ctlTopic&&controller&&Array.isArray(controller.lastPrediction)&&controller.lastPrediction.length){
      const pred=controller.lastPrediction,us=controller.lastControls||[];drawSeries(canvas,[{values:pred.map(function(x){return x[2]*180/Math.PI;}),color:'#14845d'},{values:us,color:'#7659b0',dash:[4,3]}]);return;
    }
    if(topic==='mhe'&&trace.length){drawSeries(canvas,[{values:trace.map(q=>q.observerCost||0),color:'#7659b0'},{values:trace.map(q=>q.observerIterations||0),color:'#278b98',dash:[4,3]}]);return;}
    const rows=trace.filter(function(q){return q.P;});if(rows.length){const sv=rows.map(function(q){return Math.sqrt(Math.max(0,q.P[1][1]));}),sw=rows.map(function(q){return Math.sqrt(Math.max(0,q.P[3][3]));}),rel=rows.map(function(q){return q.reliability?Math.min.apply(null,q.reliability):0;}),ss=[{values:sv,color:'#7659b0'},{values:sw,color:'#278b98'}];if(rows.some(function(q){return q.reliability;}))ss.push({values:rel,color:'#14845d',dash:[4,3]});drawSeries(canvas,ss);return;}
    const g=canvas.getContext('2d');g.clearRect(0,0,canvas.width,canvas.height);g.fillStyle='#7b8d9c';g.font='11px system-ui';
    g.fillText($('observer').value==='residual'?'보정 출력에는 calibrated P가 없습니다 · base EKF P만 내부 유지':'이 모드는 output covariance P를 제공하지 않습니다.',10,24);
  }
  function renderMetrics(){
    $('clock').textContent='t='+(plant.steps*L.DT).toFixed(2)+' s';$('mTheta').textContent=(plant.s[2]*180/Math.PI).toFixed(2)+'°';$('mThetaHat').textContent=(lastEstimate[2]*180/Math.PI).toFixed(2)+'°';$('mRmse').textContent=Math.sqrt(sumErr/Math.max(1,nErr)).toFixed(3);$('mForce').textContent=lastForce.toFixed(2)+' / '+lastAppliedForce.toFixed(2)+' N';$('mInnov').textContent=Math.hypot.apply(null,lastInnovation).toFixed(3);
    $('mSolve').textContent=controller&&Number.isFinite(controller.lastSolveMs)&&controller.lastSolveMs>0?controller.lastSolveMs.toFixed(2)+' ms':'—';$('mIter').textContent=controller&&controller.lastPrediction&&controller.lastPrediction.length?String(controller.lastPrediction.length-1)+' / '+String(controller.lastIterations??0):'—';$('statusBadge').textContent=running?'RUNNING':'PAUSED';$('play').textContent=running?'일시정지':'재생';
  }
  function updatePipeline(){$('pipeSensor').textContent='y=['+lastMeasurement.map(function(v){return v.toFixed(2);}).join(',')+']';$('pipeObserver').textContent=observerLabels[$('observer').value];$('pipeController').textContent=controllerLabels[$('controller').value];}
  function renderAll(){world();stateChart();controlChart();contextChart();renderMetrics();updatePipeline();}
  function loop(ts){if(!lastTs)lastTs=ts;const dt=Math.min(.08,(ts-lastTs)/1000);lastTs=ts;if(running){acc+=dt;let n=0;while(acc>=L.DT&&n<4){stepOne();acc-=L.DT;n++;}}renderAll();requestAnimationFrame(loop);}
  function compare(){
    const sc=$('compareScenario').value,cs=['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','ppo'],os=['truth','raw','kf','ekf','so2','residual','adaptive'];let html='<thead><tr><th>Controller</th>'+os.map(function(o){return '<th>'+observerLabels[o]+'</th>';}).join('')+'</tr></thead><tbody>';
    cs.forEach(function(c){html+='<tr><td>'+controllerLabels[c]+'</td>';os.forEach(function(o){const r=L.runEpisode({controller:c,observer:o,scenario:sc,seed:77,steps:240,goal:+$('goal').value,actor:actor,residualModel:residualModel,pushAt:96,pushForce:3});html+="<td class='"+(r.failed?'fail':'pass')+"'>"+(r.failed?'FAIL':'✓')+' · '+r.rmseState.toFixed(2)+(r.meanSolveMs?'<br><small>'+r.meanSolveMs.toFixed(1)+'ms</small>':'')+'</td>';});html+='</tr>';});$('matrix').innerHTML=html+'</tbody>';
  }
  function statePayload(){
    const Pout=observer.outputCovariance?observer.outputCovariance():observer.P;
    return {topic:topic,controller:$('controller').value,observer:$('observer').value,scenario:$('scenario').value,t:plant.steps*L.DT,goal:plant.goal,
      truth:plant.s.slice(),estimate:lastEstimate.slice(),measurement:lastMeasurement.slice(),innovation:lastInnovation.slice(),force:lastForce,appliedForce:lastAppliedForce,
      P:Pout?Pout.map(function(r){return r.slice();}):null,baseP:observer.P?observer.P.map(function(r){return r.slice();}):null,
      covarianceScope:Pout?'output-estimate':'base-filter-only-or-not-applicable',
      solver:{solveMs:controller.lastSolveMs||0,iterations:controller.lastIterations||0,horizon:controller.lastPrediction&&controller.lastPrediction.length?controller.lastPrediction.length-1:0,cost:controller.lastCost===undefined?null:controller.lastCost},
      rmse:Math.sqrt(sumErr/Math.max(1,nErr)),running:running};
  }
  async function registerWebMCP(){
    const mc=document.modelContext||navigator.modelContext;if(!mc||!mc.registerTool){$('webmcpBadge').textContent='WebMCP unavailable · UI 정상';$('webmcpBadge').classList.add('warn');return;}const ac=new AbortController();window.__controlLabV2Abort=ac;const ro={readOnlyHint:true,untrustedContentHint:false},rw={readOnlyHint:false,untrustedContentHint:false};
    const reg=async function(tool){return mc.registerTool(tool,{signal:ac.signal});};
    try{
      await reg({name:'cartpole_get_state',description:'Read the detailed CartPole control-observer lab state and solver metrics.',inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:ro,execute:async function(){return JSON.stringify(statePayload());}});
      await reg({name:'cartpole_set_controller',description:'Select a controller.',inputSchema:{type:'object',properties:{controller:{type:'string',enum:['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','ppo']}},required:['controller'],additionalProperties:false},annotations:rw,execute:async function(a){$('controller').value=a.controller;rebuild();return 'controller='+a.controller;}});
      await reg({name:'cartpole_set_observer',description:'Select an observer.',inputSchema:{type:'object',properties:{observer:{type:'string',enum:['truth','raw','kf','ekf','so2','mhe','residual','adaptive']}},required:['observer'],additionalProperties:false},annotations:rw,execute:async function(a){$('observer').value=a.observer;rebuild();return 'observer='+a.observer;}});
      await reg({name:'cartpole_set_topic',description:'Open an educational topic without changing plant state.',inputSchema:{type:'object',properties:{topic:{type:'string',enum:ORDER}},required:['topic'],additionalProperties:false},annotations:rw,execute:async function(a){setTopic(a.topic);return 'topic='+a.topic;}});
      await reg({name:'cartpole_set_scenario',description:'Select scenario.',inputSchema:{type:'object',properties:{scenario:{type:'string',enum:['nominal','sensor','model','mixed','bias','actuator','latency','nonlinear','sim2real','glitch']}},required:['scenario'],additionalProperties:false},annotations:rw,execute:async function(a){$('scenario').value=a.scenario;rebuild();return 'scenario='+a.scenario;}});
      await reg({name:'cartpole_apply_push',description:'Apply bounded external force.',inputSchema:{type:'object',properties:{force:{type:'number',minimum:-8,maximum:8},steps:{type:'integer',minimum:1,maximum:50}},required:['force'],additionalProperties:false},annotations:rw,execute:async function(a){plant.applyPush(a.force,a.steps||10);return 'push='+a.force;}});
      await reg({name:'cartpole_reset',description:'Reset experiment.',inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:rw,execute:async function(){rebuild();return 'reset';}});
      await reg({name:'cartpole_run_steps',description:'Advance live experiment.',inputSchema:{type:'object',properties:{steps:{type:'integer',minimum:1,maximum:500}},required:['steps'],additionalProperties:false},annotations:rw,execute:async function(a){for(let i=0;i<a.steps;i++)stepOne();renderAll();return JSON.stringify(statePayload());}});
      await reg({name:'cartpole_run_probe',description:'Run deterministic offline controller-observer probe without changing live state.',inputSchema:{type:'object',properties:{controller:{type:'string',enum:['pid','lqr','linear_mpc','scenario_mpc','state_mpc','ltv_mpc','centroidal_mpc','full_nmpc','ppo']},observer:{type:'string',enum:['truth','raw','kf','ekf','so2','mhe','residual','adaptive']},scenario:{type:'string',enum:['nominal','sensor','model','mixed','bias','actuator','latency','nonlinear','sim2real','glitch']},seed:{type:'integer',minimum:1,maximum:1000000},steps:{type:'integer',minimum:20,maximum:500},pushForce:{type:'number',minimum:-8,maximum:8}},required:['controller','observer','scenario'],additionalProperties:false},annotations:ro,execute:async function(a){return JSON.stringify(L.runEpisode({controller:a.controller,observer:a.observer,scenario:a.scenario,seed:a.seed||77,steps:a.steps||300,goal:+$('goal').value,actor:actor,residualModel:residualModel,pushAt:Math.min(120,Math.floor((a.steps||300)*.4)),pushForce:a.pushForce===undefined?(a.scenario==='nonlinear'?0:3):a.pushForce}));}});
      $('webmcpBadge').textContent='WebMCP tools registered';$('webmcpBadge').classList.add('ok');
    }catch(e){console.warn(e);$('webmcpBadge').textContent='WebMCP registration failed';$('webmcpBadge').classList.add('warn');}
  }
  $('useTopic').addEventListener('click',useTopic);['controller','observer','scenario','goal'].forEach(function(id){$(id).addEventListener('change',rebuild);});$('play').addEventListener('click',function(){running=!running;renderMetrics();});$('step').addEventListener('click',function(){running=false;stepOne();renderAll();});$('reset').addEventListener('click',rebuild);$('pushL').addEventListener('click',function(){plant.applyPush(-3,10);});$('pushR').addEventListener('click',function(){plant.applyPush(3,10);});$('compare').addEventListener('click',compare);
  window.addEventListener('hashchange',function(){const k=(location.hash||'#overview').slice(1);if(TOPICS[k]){topic=k;renderTopic();}});
  $('controller').value='lqr';$('observer').value='kf';$('scenario').value='nominal';$('goal').value='0';renderTopic();rebuild();registerWebMCP();requestAnimationFrame(loop);
})();