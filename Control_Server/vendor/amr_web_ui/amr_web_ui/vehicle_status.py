"""Convert the typed status contract to the existing ROS-free HTTP display format."""
import math


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def display_snapshot(msg, transport_age):
    result = {'vehicle':dict(age=transport_age, valid=True, fresh=True, hz=None,
        session=msg.publisher_session,sequence=msg.sequence,epoch=msg.epoch,
        mode=msg.control_mode if msg.control_mode_received else 'UNKNOWN',
        navigation=msg.navigation_state if msg.navigation_state_received else 'UNKNOWN')}

    def sample(name, meta, **values):
        if meta.received:
            result[name] = dict(age=meta.age_sec+transport_age,valid=meta.valid,fresh=meta.fresh,
                                timeout=meta.timeout_sec,hz=meta.rate_hz if meta.rate_valid else None,**values)
            if any(isinstance(v,float) and not math.isfinite(v) for v in values.values()):
                result[name]['valid'] = result[name]['fresh'] = False
                for key,value in list(result[name].items()):
                    if isinstance(value,float) and not math.isfinite(value):
                        result[name][key] = None

    for name in ('scan','imu'):
        sample(name,getattr(msg,name))
    for name in ('wheel','ekf','command'):
        twist = getattr(msg,name+'_velocity')
        sample(name,getattr(msg,name),linear=twist.linear.x,angular=twist.angular.z)
    sample('joints',msg.joints,left_rpm=msg.left_motor.wheel_rpm,right_rpm=msg.right_motor.wheel_rpm)
    for side in ('left','right'):
        motor = getattr(msg,side+'_motor')
        sample('motor_'+side+'_heartbeat',motor.heartbeat,error_code=motor.error_code,axis_state=motor.axis_state)
        sample('motor_'+side+'_power',motor.power,voltage=motor.input_voltage_v,current=motor.bus_current_a)
        sample('motor_'+side+'_temperature',motor.temperature,temperature=motor.driver_temperature_c)
    sample('diagnostics',msg.diagnostics,level=msg.diagnostic_level,detail=msg.diagnostic_detail,
        counters=dict(received=str(msg.received_frames),invalid=str(msg.invalid_frames),
                      stale_frames=str(msg.stale_frames),wheel_rebases=str(msg.wheel_rebases),
                      clock_resets=str(msg.clock_resets)))
    p,q = msg.map_pose.position,msg.map_pose.orientation
    heading = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    sample('pose',msg.pose_status,x=p.x,y=p.y,yaw=heading)
    sample('localization',msg.localization_status,state=msg.localization_state,
           ready=msg.localization_ready,detail=msg.localization_detail,tracking_warning=msg.tracking_warning)
    sample('time',msg.time_status,healthy=msg.time_healthy,latched=msg.time_fault_latched,
           detail=msg.time_detail)
    sample('battery',msg.battery_status,soc=msg.battery_soc_percent if msg.battery_soc_available else None,
           voltage=msg.battery_voltage_v if msg.battery_voltage_available else None,
           current=msg.battery_current_a if msg.battery_current_available else None,
           temperature=msg.battery_temperature_c if msg.battery_temperature_available else None,
           charge_state=msg.battery_charge_state)
    distance = dict(value=msg.wheel_distance_m,epoch=msg.wheel_distance_epoch,
                    valid=msg.wheel_distance_valid and math.isfinite(msg.wheel_distance_m))
    return result,distance
