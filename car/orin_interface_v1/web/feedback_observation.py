"""Logging-only feedback contract. No device access or control decisions.

MCU command registers are NOT measured steering angle or motor PWM waveform.
The cached status captured before dispatch is NOT that dispatch's response.
"""
import math

VERSION=1

def bounded_context(context):
    if not isinstance(context,dict):return None
    numeric=('pid_input_seq','frame_seq','capture_monotonic_s','consumed_monotonic_s',
             'image_error','control_error','steering_proposal_us','pwm_proposal_pct')
    text=('run_id','path_algorithm','path_revision','controller_revision','route_state_estimate',
          'route_reason_estimate','road_source','scenario_label_source','context_error')
    result={k:finite(context.get(k)) for k in numeric}
    result.update({k:context[k][:160] if isinstance(context.get(k),str) else None for k in text})
    result['scenario_label']=None
    return result

def finite(value):
    return value if type(value) in (int,float) and math.isfinite(value) else None

def vision_context(run_id, sequence, vision, proposal):
    """Small serializable link; no masks or unbounded diagnostic payloads."""
    try:
        diag=vision.get('path_diagnostic') or {}
        selection=(diag.get('line_estimate') or {}).get('straight_selection') or {}
        return dict(run_id=run_id,pid_input_seq=sequence,frame_seq=vision.get('frame_seq'),
            capture_monotonic_s=finite(vision.get('capture_at')),
            consumed_monotonic_s=finite(vision.get('consumed_at')),
            path_algorithm=vision.get('path_algorithm'),path_revision=diag.get('path_revision'),
            controller_revision=proposal.get('sampling_version'),
            image_error=finite(vision.get('error')),control_error=finite(proposal.get('error')),
            steering_proposal_us=finite(proposal.get('steer_us')),pwm_proposal_pct=finite(proposal.get('pwm')),
            route_state_estimate=selection.get('route_state'),route_reason_estimate=selection.get('reason'),
            road_source=(diag.get('road_state') or {}).get('source'),
            scenario_label=None,scenario_label_source='unlabelled; route fields are algorithm estimates')
    except (TypeError,AttributeError):
        return dict(run_id=run_id,pid_input_seq=sequence,context_error='MALFORMED_CONTEXT')

def observation(state, read_at, context, run_id, generation, command, ack_at):
    """Pure projection of an existing cache snapshot, never additional polling.

Host monotonic clocks can be subtracted within one Orin boot only. MCU ms
and encoder sample_ms remain in their own clock domain, including wraps.
"""
    try:
        v=state.get('vehicle') or {};age=finite(state.get('uart_age_s'))
        read_at=finite(read_at)
        cache_time=read_at-age if read_at is not None and age is not None and age>=0 else None
        fields={k:v.get(k) for k in ('ms','generation','state','reason','cmd_age_ms','steer_us_command',
            'steer_output_enabled','servo_returning','pca_ok','i2c_errors','enc_sample_ms','enc_enabled',
            'enc_a','enc_b','motor_a','motor_b','speed_mps')}
        missing=[k for k,val in fields.items() if val is None]
        return dict(version=VERSION,usage='record_only',controller_feedback_enabled=False,
            run_id=run_id,generation=generation,vision=context,
            timing=dict(clock_domain='orin_monotonic_same_boot',cache_read_monotonic_s=read_at,
                cache_received_estimate_monotonic_s=cache_time,uart_age_s=age,
                dispatch_ack_monotonic_s=finite(ack_at),
                association='status_observed_before_dispatch_not_command_response',
                receipt_precision='derived from cache age; not exact UART arrival timestamp',
                mcu_clock_domain='mcu_ms; not synchronized to Orin'),
            requested_command=dict(command),mcu_report=fields,
            telemetry_identity=[v.get('generation'),v.get('ms')],encoder_identity=[v.get('generation'),v.get('enc_sample_ms')],
            generation_matches=v.get('generation')==generation if v.get('generation') is not None else None,
            uart_error=state.get('uart_error'),missing_fields=missing,
            physical_measurements=dict(wheel_angle_deg=None,motor_pwm_waveform=None,imu=None),
            semantics=dict(steer_us_command='MCU reported commanded pulse; not measured wheel angle',
                motor_a_b='MCU reported command/target; not electrical feedback',
                encoder='raw counters; no assumed wheel mapping, distance calibration or speed conversion',
                imu='not present in current validated logging contract'))
    except (TypeError,AttributeError,ValueError):
        return dict(version=VERSION,usage='record_only',controller_feedback_enabled=False,
            run_id=run_id,observation_error='MALFORMED_CACHE_SNAPSHOT')
