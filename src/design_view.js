// Existing-controller study UI: task, candidate budget, lock, test and explicit application.
// Every number shown is computed by DesignStudy or a declared task setting, not a mock score.
const DesignLessonView={mount({manifest:m,pause,apply}){
 const D=globalThis.DesignStudy,$=id=>document.getElementById(id),label=id=>m.pairs.find(p=>p.id===id)?.id??'없음';
 let status='idle',result=null,error=null,busy=false;
 const f=(v,d=4)=>Number.isFinite(v)?v.toFixed(d):'—';
 const cells=(table,rows)=>{const b=$(table).tBodies[0];b.replaceChildren();for(const row of rows){const tr=b.insertRow();for(const v of row)tr.insertCell().textContent=String(v);}};
 function state(){return {status,error,selection:result?{recommendedPairId:result.selection.recommendedPairId,pairs:result.selection.pairs.map(p=>({pairId:p.pairId,candidate:p.candidate,eligible:p.eligible,validationScore:p.validationScore})),testUsedForSelection:false}:null,admission:result?.admission??null,summary:result?.summary??null,defaultsChanged:false};}
 function candidateTable(){
  if(!result)return;const p=result.selection.pairs.find(p=>p.pairId===$('designPair').value);if(!p)return;
  cells('designCandidates',p.training.map(t=>{const v=p.validation.find(v=>v.candidate.id===t.candidate.id);return [t.candidate.id,t.candidate.baseline?'선언 기준':'후보',t.hardFailures+' / '+t.taskFailures,f(t.fullScore),v?(v.hardFailures+' / '+v.taskFailures):'선발 제외',f(v?.fullScore),p.candidate.id===t.candidate.id?'선택 · TEST 이전':'—'];}));
  const summary=result.summary.find(s=>s.pairId===p.pairId);
  cells('designTests',result.tests.filter(t=>t.pairId===p.pairId).map(t=>[t.caseId,t.group,t.baseline.outcome+' / '+t.candidate.outcome,(t.baseline.taskPassed?'충족':'미달')+' → '+(t.candidate.taskPassed?'충족':'미달'),f(t.baseline.fullScore)+' → '+f(t.candidate.fullScore),f(t.baseline.positionTrackingRmse,3)+' → '+f(t.candidate.positionTrackingRmse,3)+' m',t.paired.commonPrefixSteps]));
  const d=D.design(m,m.pairs.find(q=>q.id===p.pairId),p.candidate,result.capture.fit);
  $('designParameters').textContent=p.pairId+' / '+p.candidate.id+'\nQ_c=['+d.Qc.map(v=>f(v,3)).join(', ')+'], R_c='+f(d.Rc,6)+'\nQ_e=['+d.Qe.map(v=>v.toExponential(2)).join(', ')+']\nR_e(별도 측정)=['+d.Re.map(v=>v.toExponential(4)).join(', ')+']\nP_0 diag=['+d.P0.join(', ')+']\nK(LQR 기준)=['+d.K.map(v=>f(v,5)).join(', ')+']\nMPC horizon='+d.horizon+', full Riccati terminal residual='+d.terminalInfo.normalizedResidual.toExponential(2)+'\nQ_c/R_c는 요구사항 설계값, Q_e는 유효 탐색값입니다. R_e 이외를 실측 보정값이라고 부르지 않습니다.';
  $('designTestScope').textContent='TEST는 선택에 사용하지 않았습니다. 실패한 조합으로 바꿔 재추천하지 않습니다. 현재 표: '+summary.pairId+' · primary와 boundary를 합쳐 승인하지 않습니다.';
 }
 function render(){
  const selected=result.selection;
  cells('designPairs',selected.pairs.map(p=>[p.pairId,p.candidate.id,p.candidate.baseline?'기준 유지':'후보 변경',p.eligible?'과제 기준 충족':'과제 미충족',f(p.validationScore,6),p.pairId===selected.recommendedPairId?'사전 규칙 추천':'—']));
  $('designPair').innerHTML=m.pairs.map(p=>'<option value="'+p.id+'">'+p.id+'</option>').join('');$('designPair').value=selected.recommendedPairId??m.pairs[0].id;
  $('designApplyCase').innerHTML=m.test.filter(t=>t.group==='primary').map(t=>'<option value="'+t.id+'">'+t.id+'</option>').join('');
  const a=result.admission;
  $('designVerdict').textContent='VALIDATION 추천: '+label(selected.recommendedPairId)+' · TEST 판정: '+(a.accepted?'선언한 primary 과제의 명시적 재실행 허용':'채택 거부: '+a.reasons.join('; '))+'. 점수 차이 1% 이내에서는 사전 순서로 단순한 조합을 택합니다. 전역 최적·임의 조건·하드웨어 보장은 아닙니다.';
  $('designApply').disabled=!a.accepted;$('designApplyCase').disabled=!a.accepted;
  $('designStatus').textContent='완료: TRAIN '+selected.trainingEvaluations+' + VALIDATION '+selected.validationEvaluations+' + TEST '+result.tests.length*2+' 회. 새로운 숫자로 바뀌어야 성공인 것이 아닙니다. 기본값은 변경하지 않았습니다.';
  candidateTable();
 }
 function disable(value){for(const id of ['designRun','designPair','calibrationRun','processRun'])$(id).disabled=value;}
 async function run(){
  if(busy||window.calibrationLesson?.getState().status==='running')throw Error('Another study is running');
  pause();busy=true;status='running';error=null;result=null;disable(true);
  for(const id of ['designPairs','designCandidates','designTests'])cells(id,[]);
  $('designApply').disabled=true;$('designApplyCase').disabled=true;$('designVerdict').textContent='아직 추천·채택 결과가 없습니다.';$('designParameters').textContent='';
  try{
   result=await D.runStudy(m,{retainAudit:false,yieldControl:()=>new Promise(resolve=>setTimeout(resolve,0)),onProgress:e=>{
    $('designStatus').textContent=e.stage==='selection-locked'?'선택 잠금: '+label(e.recommendedPairId)+' · 아직 TEST 평가 0회':e.stage+' · '+(e.pairId??'')+' '+(e.candidateId??e.caseId??'')+' '+(e.done??'')+(e.total?' / '+e.total:'');
   }});
   status='completed';render();return state();
  }catch(e){status='error';error=String(e.message);result=null;$('designStatus').textContent='중단: '+error;$('designApply').disabled=true;throw e;}
  finally{busy=false;disable(false);}
 }
 function applySelected(){
  if(busy||!result)throw Error('No completed design study');
  const test=m.test.find(t=>t.id===$('designApplyCase').value),recipe=D.application(m,result.selection,result.admission,test,result.capture.fit);
  apply(recipe);$('designStatus').textContent='수동 적용: '+recipe.pair.id+' / '+recipe.candidate.id+' / '+test.id+' · 공통 시뮬레이션을 지정 초기 상태로 되돌리고 일시정지했습니다. 일반 설정 변경·Reset은 이 과제 설정을 해제합니다.';
  $('world').scrollIntoView({behavior:'smooth',block:'center'});return recipe;
 }
 async function loadTuningComparison(){
  const status=$('tuningComparisonStatus');status.dataset.state='loading';status.textContent='오프라인 근거를 읽는 중';$('tuningComparison').tBodies[0].replaceChildren();$('loadTuningComparison').disabled=true;
  try{
   const response=await fetch('evidence/sequential_tuning.json',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);
   const report=await response.json();if(!report.experimentValid||report.mode!=='frozen-full')throw Error('완료된 고정 비교가 아닙니다');
   const mean=a=>a.length?a.reduce((s,v)=>s+v,0)/a.length:NaN,fmt=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
   const rows=['grid_budget','random_full','random_racing','smac_racing','grid_exhaustive'].map(method=>{
    const rr=report.campaigns.filter(c=>c.method===method),effect=report.pairedStatistics.find(c=>c.method===method),ci=effect?.conditionalPairedBootstrap95;
    return [method,rr.length,fmt(mean(rr.map(r=>r.actualSearchRollouts)),0),fmt(mean(rr.map(r=>r.fullyEvaluatedConfigurations)),1),fmt(mean(rr.map(r=>r.tunerSeconds)))+' s',fmt(mean(rr.map(r=>r.searchWallSeconds)))+' s',fmt(mean(rr.map(r=>r.primaryTaskSuccesses)),1)+' / 12',effect?fmt(effect.meanRelativeTaskDifference*100)+'%'+(ci?' ['+fmt(ci[0]*100)+', '+fmt(ci[1]*100)+']':' · 구간 미확정'):method==='grid_exhaustive'?'추가 예산 유한 참조':'동일 예산 기준'];
   });cells('tuningComparison',rows);
   status.dataset.state='loaded';status.textContent='SMAC '+report.versions.smac+' / ConfigSpace '+report.versions.ConfigSpace+' · 실제 총 평가 '+report.actualEvaluations+'회 · validation/test 추가 비용은 원본 기록에 별도 표기 · 기본값 변경 없음';
  }catch(e){$('tuningComparison').tBodies[0].replaceChildren();status.dataset.state='error';status.textContent='결과 표시 중단: '+e.message;}
  finally{$('loadTuningComparison').disabled=false;}
 }
 $('loadTuningComparison').onclick=loadTuningComparison;
 $('designRun').onclick=()=>run().catch(()=>{});$('designPair').onchange=candidateTable;$('designApply').onclick=()=>{try{applySelected();}catch(e){$('designVerdict').textContent=String(e.message);}};
 $('openDesign').onclick=()=>$('designLesson').scrollIntoView({behavior:'smooth',block:'start'});
 $('designApply').disabled=true;$('designApplyCase').disabled=true;
 return {run,getState:state,applySelected};
}};
