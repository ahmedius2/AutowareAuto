import os
import sys
import math
import glob
import numpy as np
import matplotlib.pyplot as plt
from builtin_interfaces.msg import Time
from callback_profile.msg import CallbackProfile
from pathlib import Path
from rosbags.highlevel import AnyReader

from rosbags.serde import serialize_cdr
from rosbags.typesys import get_types_from_msg, register_types

from rosbags.typesys import Stores, get_typestore

msg_tuples = [
        ('src/core/autoware_adapi_msgs/autoware_adapi_v1_msgs/operation_mode/msg/OperationModeState.msg',
        'autoware_adapi_v1_msgs/msg/OperationModeState'),
        ('src/universe/external/tier4_autoware_msgs/tier4_debug_msgs/msg/Float64Stamped.msg',
         'tier4_debug_msgs/msg/Float64Stamped'),
        ('src/core/autoware_msgs/autoware_vehicle_msgs/msg/VelocityReport.msg',
        'autoware_vehicle_msgs/msg/VelocityReport'),
        ('src/core/autoware_msgs/autoware_perception_msgs/msg/DetectedObjects.msg',
         'autoware_perception_msgs/msg/DetectedObjects'),
        ('src/core/autoware_msgs/autoware_perception_msgs/msg/PredictedObjects.msg',
         'autoware_perception_msgs/msg/PredictedObjects')

]
add_types = {}
for pth, msg_name in msg_tuples:
    msg_text = Path(pth).read_text()
    add_types.update(get_types_from_msg(msg_text, msg_name))

typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

def create_entry(data_dict, stat_name, metric_name, experiment_name):
    if stat_name not in data_dict:
        data_dict[stat_name] = {'experiment': experiment_name, 'metric': metric_name,
                                'timestamps': [], 'data': []}

def add_to_dict(data_dict, stat_name, timestamp, data):
    d = data_dict[stat_name]
    d['timestamps'].append(timestamp)
    d['data'].append(data)

def read_bag(bag_file_path):
    experiment_name = bag_file_path.split('/')[-1]
    data_dict = {}

    start_ts, end_ts = None, None
    # create reader instance and open for reading
    with AnyReader([Path(bag_file_path)], default_typestore=typestore) as reader:
        #connections = [x for x in reader.connections if x.topic == topic]

        connections = reader.connections
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if 'time_ms' in connection.topic:
                key = connection.topic + '_ms'
                create_entry(data_dict, key, 'Milliseconds', experiment_name)

                msg = reader.deserialize(rawdata, connection.msgtype)
                exec_time_ms = msg.data
                add_to_dict(data_dict, key, float(timestamp) * 1e-9, exec_time_ms)
            elif 'metrics' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                for stat in msg.status:
                    create_entry(data_dict, stat.name, stat.name, experiment_name)
                    ts = float(timestamp) * 1e-9
                    if len(stat.values) == 1: # key is metric_value
                        data = float(stat.values[0].value)
                    elif len(stat.values) == 3: # keys are min max mean
                        data = float(stat.values[2].value)
                    add_to_dict(data_dict, stat.name, ts, data)
            elif 'operation_mode' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                mode = int(msg.mode)
                ts = float(timestamp) * 1e-9
                if mode == 2 and (start_ts is None or ts < start_ts):
                    start_ts = ts
                elif mode == 1 and (end_ts is None or ts > end_ts):
                    end_ts = ts
            elif 'velocity_status' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                ts = timestamp * 1e-9

                longtd_vel = msg.longitudinal_velocity
                lateral_vel = msg.lateral_velocity
                vel = math.sqrt(longtd_vel**2 + lateral_vel**2)

                create_entry(data_dict, 'Velocity', 'm/s', experiment_name)
                add_to_dict(data_dict, 'Velocity', ts, vel)

    assert (start_ts is not None) and (end_ts is not None)

    # Do filtering
    for dct in data_dict.values():
        timestamps =  np.array(dct['timestamps'])
        data = np.array(dct['data'])
        mask = np.logical_and((timestamps < end_ts), (timestamps > start_ts))
        dct['timestamps'] = timestamps[mask] - start_ts
        dct['data'] = data[mask]

    if 'Velocity' in data_dict:
        timestamps = np.array(data_dict['Velocity']['timestamps'])
        velocities = np.array(data_dict['Velocity']['data'])

        sort_inds = np.argsort(timestamps)
        timestamps = timestamps[sort_inds]
        velocities = velocities[sort_inds]
        
        # Calculate time differences between consecutive points
        dt = np.diff(timestamps)
        
        # Calculate acceleration (first derivative of velocity)
        accelerations = np.diff(velocities) / dt
        mean_acceleration = np.abs(accelerations).mean()
        
        # Calculate jerk (second derivative of velocity)
        jerks = np.diff(accelerations) / dt[:-1]
        mean_jerk = np.abs(jerks).mean()
        print(experiment_name, 'mean acc:', mean_acceleration, 'mean jerk:', mean_jerk)

    print(experiment_name, 'time to reach destination:', end_ts - start_ts, 'seconds')
    create_entry(data_dict, 'end_timestamp', 'seconds', experiment_name)
    add_to_dict(data_dict, 'end_timestamp', end_ts - start_ts, end_ts - start_ts)

    return data_dict

def merge_data_dicts(data_dicts):
    #determine start end timestamps
    glob_end_ts = max([dd['end_timestamp']['data'] for dd in data_dicts])[0]

    merged_data_dict = {}
    for dd in data_dicts:
        for stat_name, dct in dd.items():
            if stat_name != 'end_timestamp':
                if stat_name not in merged_data_dict:
                    merged_data_dict[stat_name] = []
                merged_data_dict[stat_name].append(dct) # append different experiments

    return merged_data_dict, glob_end_ts

def plot_all(merged_data_dicts, glob_end_ts, dest_dir="../../shared/aw_timing_plots/"):
    strings = []
    for stat_name, data_dicts in merged_data_dicts.items():
        fig, ax = plt.subplots(1, 1, figsize=(12, 3), constrained_layout=True)
        if stat_name[-3:] == '_ms':
            fields = stat_name.split('/')[1:]
            assert len(fields) > 1
            target_dir = dest_dir + "/".join(fields[:-2])
            os.makedirs(target_dir, exist_ok=True)
            fname =  fields[-2] + '-' + fields[-1]
        
            # Plot the data
            for dd in data_dicts:
                timestamps = dd['timestamps']
                exec_times = dd['data']
                experiment_name = dd['experiment']
                ax.scatter(timestamps, exec_times, label=experiment_name, s=2)
            ax.set_xlabel('Timestamps (sec)')
            ax.set_ylabel(data_dicts[0]['metric'])
            ax.set_title(fname)
            ax.grid('True', ls='--')
            ax.legend()
            ax.set_xlim(0, glob_end_ts)
            plt.savefig(target_dir + "/" + fname)
            plt.close(fig)
        else:
            target_dir = "../../shared/aw_timing_plots"
            for dd in data_dicts:
                timestamps = dd['timestamps']
                data = dd['data']
                experiment_name = dd['experiment']
                strings.append(f"{experiment_name} mean {stat_name}: {np.mean(data)}")    
                ax.scatter(timestamps, data, label=experiment_name, s=2)
            ax.set_xlabel('Timestamps (sec)')
            ax.set_ylabel(f'{stat_name}')
            #mindata, maxdata = np.min(data), np.max(data)
            #ax.set_yticks(np.arange(mindata, maxdata, (maxdata-mindata)/10))
            ax.set_xlim(0, glob_end_ts)
            ax.legend()
            #ax.grid('True', ls='--')
            pth = target_dir + "/" + stat_name + '.png'
            plt.savefig(pth)
            plt.close(fig)
 
    for s in sorted(strings):
        print(s)

if __name__ == '__main__':
    data_dicts = []

    paths = sorted(glob.glob(sys.argv[1] + '/*'))
    for pth in paths:
        print(pth)
        data_dicts.append(read_bag(pth))
    merged_data_dicts, glob_end_ts = merge_data_dicts(data_dicts)
    plot_all(merged_data_dicts, glob_end_ts)

