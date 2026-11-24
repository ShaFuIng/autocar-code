/* Only decides whether an existing preview can be reused; never sends commands. */
(function(root){
 const api={
  reusable(s,d,model,path){return !!s && ['loading','ready','no_reliable_path','camera_unavailable'].includes(s.state) && s.model_id===model && s.path_algorithm===path && (s.curve_outward_px_640??0)===(d?.data?.parameters?.curve_outward_px_640??0) && s.row_ratio===d?.data?.parameters?.row_ratio && s.profile?.center_us===d?.data?.parameters?.center_us;},
  label(s){return s.state==='idle' ? (s.preview_unload_reason==='IDLE_TIMEOUT'?'模型已因閒置卸載；連線正常':s.preview_unload_reason==='USER_STOP'?'模型已手動停止；連線正常':'模型未載入／已卸載；連線正常') : '連線正常';}
 };root.PIDSession=api;if(typeof module!=='undefined')module.exports=api;
})(globalThis);
