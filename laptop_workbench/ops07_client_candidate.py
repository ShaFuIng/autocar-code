"""Transport-injected 07 client; no SSH import, background work or auto retries.

For a later deployment, bind request to the existing authenticated RPC after
installing the reviewed helper allowlist. Unit tests bind an in-memory endpoint.
"""
from ops07_runtime import REVISION


class Client:
    def __init__(self,request,package):
        self.request=request;self.package=package;self.state=None;self.unknown=False

    def refresh(self):
        state=self.request('/api/ops07/status')
        if state.get('revision')!=REVISION or state.get('package_identity')!=self.package['identity']:
            raise ValueError('車端版本／方案與選擇不符，尚未完成方案部署讀回。')
        self.state=state;self.unknown=False
        return state

    def command(self,action,**payload):
        if action not in ('load','arm-hold','release','pause','resume','stop','reset'):raise ValueError('無效操作。')
        if self.unknown:raise ValueError('上一個指令回覆未知；先讀取車端狀態，不自動重送。')
        if self.state is None:raise ValueError('請先核對車端版本與方案。')
        if set(payload)&{'parameters','seconds','parameter_revision','execution_mode'}:raise ValueError('07 參數由方案固定。')
        if action in ('release','pause','stop','resume'):
            payload.update(token=self.state.get('token'),session_id=self.state.get('id'))
        try:
            result=self.request('/api/ops07/'+action,payload)
        except (OSError,TimeoutError,ValueError):
            self.unknown=True;raise
        if result.get('ok') is False:raise ValueError(result.get('reason','車端拒絕操作。'))
        if 'revision' in result:self.state=result
        return result
