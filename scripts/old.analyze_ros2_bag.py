import os
import sys
import math
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

def plot_data(bag_file_path):
    timing_dict = {} # both x and y values
    metrics_dict = {'velocity':([], [])}

    min_ts, max_ts = None, None
    # create reader instance and open for reading
    with AnyReader([Path(bag_file_path)], default_typestore=typestore) as reader:
        #connections = [x for x in reader.connections if x.topic == topic]
        connections = reader.connections
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if 'time_ms' in connection.topic:
                if connection.topic not in timing_dict:
                    timing_dict[connection.topic] = ([], []) # timestamp, exec_time_ms
  
                msg = reader.deserialize(rawdata, connection.msgtype)
                exec_time_ms = msg.data
      
                tpl = timing_dict[connection.topic]
                tpl[0].append(float(timestamp) * 1e-9)
                tpl[1].append(exec_time_ms)
            elif 'metrics' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
      
                for stat in msg.status:
                    name = stat.name
                    if name not in metrics_dict:
                        metrics_dict[name] = ([], [])
                    ts = float(timestamp) * 1e-9
                    metrics_dict[name][0].append(ts)
                    if len(stat.values) == 1: # key is metric_value
                        metrics_dict[name][1].append(float(stat.values[0].value))
                    elif len(stat.values) == 3: # keys are min max mean
                        metrics_dict[name][1].append(float(stat.values[2].value))
            elif 'operation_mode' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                mode = int(msg.mode)
                ts = float(timestamp) * 1e-9
                if mode == 2 and (min_ts is None or ts < min_ts):
                    min_ts = ts
                elif mode == 1 and (max_ts is None or ts > max_ts):
                    max_ts = ts
            elif 'velocity_status' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                ts = timestamp * 1e-9

                longtd_vel = msg.longitudinal_velocity
                lateral_vel = msg.lateral_velocity
                vel = math.sqrt(longtd_vel**2 + lateral_vel**2)

                metrics_dict['velocity'][0].append(ts)
                metrics_dict['velocity'][1].append(vel)

    assert (min_ts is not None) and (max_ts is not None)
    print('Time limit:', min_ts, max_ts)
    print('Time to reach destination:', max_ts - min_ts)

    for topic, data_tuple in timing_dict.items():
        fig, ax = plt.subplots(1, 1, figsize=(12, 3), constrained_layout=True)
        fields = topic.split('/')[1:]
        assert len(fields) > 1
        target_dir = "../../shared/aw_timing_plots/" + "/".join(fields[:-2])
        os.makedirs(target_dir, exist_ok=True)
        fname =  fields[-2] + '-' + fields[-1]
    
        # Plot the data
        timestamps = np.array(data_tuple[0])
        mask = np.logical_and((timestamps < max_ts), (timestamps > min_ts))
        timestamps = timestamps[mask]
        timestamps -= min_ts
        print(topic, timestamps.shape)
        exec_times =  np.array(data_tuple[1])[mask]
    
        ax.scatter(timestamps, exec_times, color='b')
        ax.set_xlim(0, max_ts - min_ts)
        ax.set_xlabel('Timestamps (sec)')
        ax.set_ylabel('Execution time (millisec)')
        ax.set_title(fname)
        ax.grid('True', ls='--')
        plt.savefig(target_dir + "/" + fname)
        plt.close(fig)

    strings = []
    for metric_name, data_tuple in metrics_dict.items():
        fig, ax = plt.subplots(1, 1, figsize=(12, 3), constrained_layout=True)
        target_dir = "../../shared/aw_timing_plots/metrics"
        os.makedirs(target_dir, exist_ok=True)
        fname = metric_name
    
        # Plot the data
        timestamps = np.array(data_tuple[0])
        mask = np.logical_and((timestamps < max_ts), (timestamps > min_ts))
        timestamps = timestamps[mask]
        timestamps -= min_ts
        print(metric_name, timestamps.shape)
        #sort_inds = np.argsort(timestamps)
        #timestamps = timestamps[sort_inds]
        data = np.array(data_tuple[1])[mask]
        strings.append(f"Mean {metric_name}: {np.mean(data)}")
    
        ax.scatter(timestamps, data, color='r')
        ax.set_xlabel('Timestamps (sec)')
        ax.set_ylabel(f'{metric_name}')
        #mindata, maxdata = np.min(data), np.max(data)
        #ax.set_yticks(np.arange(mindata, maxdata, (maxdata-mindata)/10))
        ax.set_xlim(0, max_ts - min_ts)
        #ax.set_title(fname)
        #ax.grid('True', ls='--')
        pth = target_dir + "/" + fname + '.png'
        plt.savefig(pth)
        plt.close(fig)
 
    for s in sorted(strings):
        print(s)

if __name__ == '__main__':
    plot_data(sys.argv[1])
