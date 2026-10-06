// Event.code works with non-English keyboard layouts. Charge time is measured
// by simulation, never by OS key-repeat cadence.
export function createGameInput({onChargeStart=()=>{},onChargeRelease=()=>{},onCancel=()=>{},onPause=()=>{},onReset=()=>{},onHelp=()=>{}}={}){
 const held=new Set(),keys=new Set(['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE','Space','KeyP','KeyR','Escape']);
 const editable=e=>/^(INPUT|TEXTAREA|SELECT|BUTTON)$/.test(e.target?.tagName??'')||e.target?.isContentEditable;
 const axes=()=>({horizontal:Number(held.has('KeyD'))-Number(held.has('KeyA')),vertical:Number(held.has('KeyW'))-Number(held.has('KeyS')),tilt:Number(held.has('KeyE'))-Number(held.has('KeyQ'))});
 function clear(){held.clear();onCancel();}
 return {axes,clear,down(e){
  if(editable(e)||e.ctrlKey||e.altKey||e.metaKey||!keys.has(e.code))return false;
  e.preventDefault?.();if(e.repeat||held.has(e.code))return true;
  if(e.code==='KeyP'){clear();onPause();return true;}if(e.code==='KeyR'){clear();onReset();return true;}if(e.code==='Escape'){clear();onHelp();return true;}
  held.add(e.code);if(e.code==='Space')onChargeStart();return true;
 },up(e){
  if(!held.has(e.code))return false;if(editable(e)){clear();return true;}held.delete(e.code);if(!editable(e))e.preventDefault?.();
  if(e.code==='Space')onChargeRelease();return true;
 }};
}
