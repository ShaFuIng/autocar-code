"""c1lap1b common loader. frames from extract.py output; heading by integrating proposal.inputs.gz (deg/s, left +) over proposal.t."""
import json, math
from pathlib import Path
SCR = Path(r'C:/Users/User/AppData/Local/Temp/claude/1eeee7db7bd1a8b1/02d05eda-955f-4b7f-90df-1bfa92030e70/scratchpad/c1b')
def num(x): return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
def g(d, *ks):
    for k in ks:
        if not isinstance(d, dict): return None
        d = d.get(k)
    return d
def load(run):
    F = []
    for l in open(SCR / f'{run}.jsonl', encoding='utf8'):
        d = json.loads(l); p = d['p']; x = d['x']
        t = p.get('t') if num(p.get('t')) else x.get('capture_at')
        lp = p.get('lap_pos') or {}; lf = p.get('lap_ff') or {}; b = lf.get('band') or {}; al = p.get('arc_lat') or {}
        af = x.get('arc_fit') or {}
        f = dict(t=t, valid=p.get('valid'), reason=p.get('reason'), owner=p.get('owner'), err=p.get('image_error'), perr=p.get('error'),
                 far=g(p, 'tp', 'far'), near=g(p, 'tp', 'near'),
                 pre=p.get('steer_us'), sent=g(p, 'play_comp', 'steer_us'), pwm=p.get('pwm'), I=p.get('i'),
                 gz=g(p, 'inputs', 'gz'), v=g(p, 'yaw_inner', 'speed_mps'),
                 lap=lp.get('lap'), seg=lp.get('seg'), s_m=lp.get('s_m'), anchor=lp.get('anchor'), sa=lp.get('since_anchor_m'), odo=lp.get('odo_m'),
                 lf_curve=lf.get('curve'), lf_state=lf.get('state'), lf_x=lf.get('x_m'), lf_turn=lf.get('turned_deg'), lf_kind=lf.get('kind'), lf_tgt=lf.get('target_us'),
                 lf_steer=lf.get('steer_us'), lf_pid=lf.get('pid_steer_us'), lf_a=lf.get('a'), lf_elig=lf.get('eligible'), lf_end=lf.get('end'),
                 tb=b.get('turned_deg'), kap=b.get('kappa'), prof=b.get('prof_us'), ec=b.get('e_c'), eus=b.get('e_us'), gzus=b.get('gz_us'), top=b.get('top_us'), plat=b.get('plat'),
                 y=al.get('y_cm') if al.get('ok') else None, clr_in=al.get('clr_in_cm') if al.get('ok') else None, clr_out=al.get('clr_out_cm') if al.get('ok') else None,
                 psi=al.get('psi_deg') if al.get('ok') else None, al_zone=al.get('zone'),
                 af_ok=af.get('ok'), af_psiL=g(af, 'L', 'psi_car_deg'), af_psiR=g(af, 'R', 'psi_car_deg'), af_k=af.get('kappa_dpm'),
                 sb=g(p, 'start_bias', 'bias_us'), sb_el=g(p, 'start_bias', 'elapsed_s'),
                 ct=g(p, 'curve_trim'), ir=p.get('i_ramp'), cseg=p.get('seg'), raw=p)
        F.append(f)
    # heading: integrate gz (left +) over t; hold through frames without gz
    h = 0.; tp = None; gp = None
    for f in F:
        if num(f['t']) and tp is not None and num(gp): h += gp * (f['t'] - tp)
        if num(f['t']): tp = f['t']
        if num(f['gz']): gp = f['gz']
        f['hd'] = h
    return F
def lap1(F):
    return [f for f in F if f['lap'] in (0, None)] if F else []
