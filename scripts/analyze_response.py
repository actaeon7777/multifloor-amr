#!/usr/bin/env python3
"""Read rosbag2 response recordings. Outputs diagnostics, never auto-edits physics."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def analyze(bag):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=''),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    channels=defaultdict(list);clock=None
    wanted={'/clock','/cmd_vel','/wheel/odometry','/imu/data_raw','/odometry/filtered'}
    while reader.has_next():
        topic,data,received=reader.read_next()
        if topic not in wanted:continue
        msg=deserialize_message(data,get_message(types[topic]))
        if topic=='/clock':clock=msg.clock.sec+msg.clock.nanosec/1e9;continue
        stamp=clock if clock is not None else received/1e9
        if hasattr(msg,'header'):stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if topic=='/cmd_vel':values=[msg.linear.x,msg.angular.z]
        elif topic=='/imu/data_raw':values=[msg.angular_velocity.z,msg.linear_acceleration.x]
        else:values=[msg.twist.twist.linear.x,msg.twist.twist.angular.z]
        channels[topic].append([stamp,*values])
    report={'bag':str(bag),'calibration_status':'diagnostic_only_not_validated',
            'trajectory_basis':'localization_estimate_only',
            'interpretation':'Delay includes actuator, telemetry and filtering. Headerless cmd_vel uses bag reception time (latest /clock in simulation).',
            'channels':{},'apparent_response_delay_s':{}}
    for name,values in channels.items():
        a=np.asarray(values);dt=np.diff(a[:,0]);positive=dt[dt>0]
        report['channels'][name]={'samples':len(a),'duration_s':float(a[-1,0]-a[0,0]),
            'median_period_s':float(np.median(positive)) if len(positive) else None,
            'p99_period_s':float(np.quantile(positive,.99)) if len(positive) else None,
            'nonmonotonic_timestamps':int((dt<=0).sum())}
    if '/cmd_vel' in channels and '/wheel/odometry' in channels:
        cmd=np.asarray(channels['/cmd_vel']);wheel=np.asarray(channels['/wheel/odometry'])
        if not np.any(np.diff(cmd[:,0])<0):
            # Several wall-clock control publications can share one /clock tick.
            # Retain the last command at that instant; do not invent sub-tick times.
            keep=np.r_[np.diff(cmd[:,0])>0,True]
            report['collapsed_same_stamp_commands']=int((~keep).sum())
            cmd=cmd[keep]
        for axis,label in [(1,'linear'),(2,'angular')]:
            if len(cmd)<20 or len(wheel)<20 or np.std(cmd[:,axis])<.02:
                report['apparent_response_delay_s'][label]=None;continue
            if np.any(np.diff(cmd[:,0])<=0) or np.any(np.diff(wheel[:,0])<=0):
                report['apparent_response_delay_s'][label]={'valid':False,'reason':'Repeated/backward timestamps; split resets or resolve clock quantization before fitting'}
                continue
            scores=[]
            for delay in np.arange(0.,.801,.01):
                t=wheel[:,0]-delay;mask=(t>=cmd[0,0])&(t<=cmd[-1,0])
                indices=np.clip(np.searchsorted(cmd[:,0],t[mask],side='right')-1,0,len(cmd)-1)
                score=np.mean((cmd[indices,axis]-wheel[mask,axis])**2) if mask.sum()>=20 else np.inf
                scores.append((float(score),float(delay)))
            score,delay=min(scores)
            report['apparent_response_delay_s'][label]={'least_square_shift_s':delay,'residual_rms':score**.5,
                'note':'Not a separated transport-delay estimate; ramp dynamics also affect this shift.'}
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('bag',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.write_text(json.dumps(analyze(args.bag),indent=2)+'\n')
