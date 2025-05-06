import argparse
import datetime
import os
import mmap
import posix_ipc
import struct
import time
import json
import queue
import torch
import math
import numpy as np
#from pathlib import Path
import sys

sys.path.insert(0, '/storage/ahmet/Anytime-Lidar')

from pcdet.utils.ros2_utils import get_dataset, pred_dict_to_f32_multi_arr, f32_multi_arr_to_detected_objs
from pcdet.config import cfg, cfg_from_list, cfg_from_yaml_file, log_config_to_file
from pcdet.models import build_network 

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.clock import ClockType, Clock
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header, MultiArrayDimension
from autoware_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification
from autoware_vehicle_msgs.msg import VelocityReport
from valo_msgs.msg import Float32MultiArrayStamped
from torch.profiler import profile, record_function, ProfilerActivity
from tier4_debug_msgs.msg import Float64Stamped, Int64Stamped
#from callback_profile.msg import CallbackProfile

# NOTE, set LIDAR_DNN_MODEL env variable
PROFILE = False
PUB_DEBUG_DETS = False
DYN_RES = False
SIM_EXEC_TIME = True

def inv_deadline_mapping(value, in_min=0, in_max=12, dl_min=93, dl_max=200):
    dl_range = (dl_max - dl_min)
    in_range = (in_max - in_min)
    return dl_max - value / in_range * dl_range

# has to be converted to [N, 6]
def f32_multi_arr_to_tensor(float_arr):
    num_points = float_arr.array.layout.dim[0].size;
    num_attr = float_arr.array.layout.dim[1].size;
    np_arr = np.array(float_arr.array.data, dtype=np.float32)
    np_arr = np_arr.reshape((num_points,num_attr))
    points = torch.from_numpy(np_arr)
    return points

class InferenceNode(Node):
    def __init__(self,  period_sec):
        super().__init__('lidar_objdet_valor')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('model_res_idx', -1)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Keep only the latest message
        )

        self.delegate_gt = int(os.environ.get("LIDAR_DNN_DELEGATE_GT", 0)) > 0

        self.period_sec = period_sec
        self.mutex = None # for shared mem

        self.pc_sub = self.create_subscription(Header,
                'accumulated_pc_as_arr', self.pc_callback, qos_profile=sensor_qos)
        if self.delegate_gt:
            self.latest_gt_queue = list()
            self.det_pub = self.create_publisher(DetectedObjects,
                    'valor_detected_objs', 10)
            self.gt_sub = self.create_subscription(DetectedObjects,
                    '/awsim/ground_truth/perception/object_recognition/detection/objects',
                    self.gt_callback, qos_profile=sensor_qos)
            return

        self.arr_pub = self.create_publisher(Float32MultiArrayStamped,
                'valor_detected_objs', 10)
        self.processing_time_pub = self.create_publisher(Float64Stamped,
                '~/exec_time_ms', 10)
        self.res_idx_pub = self.create_publisher(Int64Stamped,
                '~/selected_res_idx', 10)
        if PUB_DEBUG_DETS:
            self.det_pub = self.create_publisher(DetectedObjects,
                    '~/debug/valor_detected_obj', 10)
        self.velo_sub = self.create_subscription(VelocityReport,
                '/vehicle/status/velocity_status', self.vel_callback, 1)

        self.model_initialized = False
        self.init_model()
        self.model_initialized = True

        self.past_publish_times = np.zeros(10)
        self.vel_limit = 12 # m/s
        self.sample_counter = 0

        # Define the transformation matrix from
        # velodyne_top frame to base_link frame
        # This transformation is STATIC
        self.vt_to_bl_tf = torch.tensor([
			[  0.032, -0.999,  0.015,  0.901],
			[  0.999,  0.032,  0.000,  0.000],
			[ -0.001,  0.015,  1.000,  2.066],
			[  0.000,  0.000,  0.000,  1.000]
        ], dtype=torch.float32)

        self.publish_time_mntc = time.monotonic()

        self.shm_attached = False

    def gt_callback(self, gt_objects):
        self.latest_gt_queue.append(gt_objects)
        if len(self.latest_gt_queue) > 20:
            self.latest_gt_queue.pop(0)

    def attach_to_shm(self):
        shm_name = "/valor_pointcloud_data"
        num_fields = 4 # xyzt
        data_size =  500000 * num_fields * 4 # assuming float is 4 bytes

        self.shm_fd = os.open(f"/dev/shm{shm_name}", os.O_RDONLY)
        self.shared_pc_data = mmap.mmap(self.shm_fd, data_size, mmap.MAP_SHARED, mmap.PROT_READ)

        # Initialize in your node
        mutex_name = "/valor_data_mutex"
        self.mutex = posix_ipc.Semaphore(mutex_name)

        self.context.on_shutdown(self.on_shutdown_cb)

    def on_shutdown_cb(self):
        if self.mutex is not None:
            self.mutex.close()
            self.shared_pc_data.close()
            os.close(self.shm_fd)

    def transform_to_base_link(self, objects):
        # Extract translation and orientation components
        translations = objects[:, :3]  # [x, y, z]

        # Apply transformation to translations
        translations_h = torch.cat([translations, torch.ones(translations.shape[0], 1)], dim=1)
        transformed_translations_h = torch.mm(self.vt_to_bl_tf, translations_h.T)
        objects[:, :3] = transformed_translations_h[:3, :].T  # Extract x, y, z

        # Update yaw based on rotation
        yaw_rotation_angle = torch.atan2(self.vt_to_bl_tf[1, 0], self.vt_to_bl_tf[0, 0])  # Extract yaw rotation
        objects[:, 6] += yaw_rotation_angle

        return objects

    def init_model(self):
        pth = os.environ["PCDET_PATH"]
        os.chdir(os.path.join(pth, 'tools'))

        chosen_model = os.environ.get("LIDAR_DNN_MODEL", "MURAL_Pillarnet")

        if chosen_model == "Pillarnet0100":
            def_cfg_file  = "tools/cfgs/nuscenes_models/pillarnet0100_awsim.yaml"
            def_ckpt_file = "models/pillarnet0100_awsim_e20.pth"
        elif chosen_model == "Pillarnet0128":
            def_cfg_file  = "tools/cfgs/nuscenes_models/pillarnet0128_awsim.yaml"
            def_ckpt_file = "models/pillarnet0128_awsim_e20.pth"
        elif chosen_model == "Pillarnet0200":
            def_cfg_file  = "tools/cfgs/nuscenes_models/pillarnet0200_awsim.yaml"
            def_ckpt_file = "models/pillarnet0200_awsim_e20.pth"
        elif chosen_model == "MURAL_Pillarnet":
            def_cfg_file  = "tools/cfgs/nuscenes_models/mural_pillarnet_0100_0128_0200_awsim.yaml"
            def_ckpt_file = "models/mural_pillarnet_0100_0128_0200_awsim_e20.pth"
            global DYN_RES
            DYN_RES = True
        else:
            self.get_logger().warn(f"UNKNOWN LIDAR DNN MODEL NAME: {chosen_model}")

        def_cfg_file  = os.path.join(pth, def_cfg_file)
        def_ckpt_file = os.path.join(pth, def_ckpt_file)
        
        self.get_logger().info(f"LIDAR DNN MODEL NAME: {chosen_model}")

        cfg_from_yaml_file(def_cfg_file, cfg)
        #if args.set_cfgs is not None:
        set_cfgs = ['MODEL.METHOD', 7,
            'MODEL.DEADLINE_SEC', 10.0,
            'MODEL.DENSE_HEAD.NAME', 'CenterHeadInf',
            'MODEL.DENSE_HEAD.POST_PROCESSING.SCORE_THRESH', '0.3',
            'OPTIMIZATION.BATCH_SIZE_PER_GPU', '1']
        cfg_from_list(set_cfgs, cfg)

        logger, test_set = get_dataset(cfg)

        model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=test_set)
        model.load_params_from_file(filename=def_ckpt_file, logger=logger, to_cpu=False)
        model.eval()
        model.cuda()
        model.simulate_exec_time = SIM_EXEC_TIME
        self.model_cfg = cfg

        self.dataset = model.dataset

        oc = ObjectClassification()
        cls_names = [str(i) for i in cfg.CLASS_NAMES]
        self.cls_mapping = { cls_names.index(name)+1: oc.__getattribute__(name.upper()) \
                for name in cls_names }

        print('[IS] Calibrating...')
        torch.cuda.cudart().cudaProfilerStop()
        with torch.no_grad():
            model.calibrate()
        torch.cuda.cudart().cudaProfilerStart()

        # Remove the hooks now
        model.pre_hook_handle.remove()
        model.post_hook_handle.remove()
        model.latest_batch_dict = None
        model.last_elapsed_time_musec = 100000
        self.model = model

        dummy_tensor = torch.empty(1024**3, device='cuda')
        torch.cuda.synchronize()
        del dummy_tensor
        model.res_idx = 0 # initial resolution

    def vel_callback(self, vel_report):
        global DYN_RES
        if DYN_RES:
            longtd_vel = vel_report.longitudinal_velocity
            lateral_vel = vel_report.lateral_velocity

            vel = math.sqrt(longtd_vel**2 + lateral_vel**2)

            deadline_ms = inv_deadline_mapping(vel, 0, self.vel_limit, 100, 250)
            self.model._default_deadline_sec = deadline_ms * 1e-3
            #self.get_logger().info(f"mapped deadline ms {deadline_ms}")

#            num_res = 5
#            self.model.res_idx = round((vel / self.vel_limit) * num_res)
#            if self.model.res_idx >= num_res:
#                self.model.res_idx = num_res - 1

    def read_from_shmem(self):
        if not self.shm_attached:
            self.attach_to_shm()
            self.shm_attached = True

        self.mutex.acquire()
        self.shared_pc_data.seek(0)

        # Read the header (first two floats)
        header_bytes = self.shared_pc_data.read(8)  # 2 floats, 4 bytes each
        num_points, num_fields = struct.unpack('ff', header_bytes)
        num_points = int(num_points)
        num_fields = int(num_fields)
        
        # Calculate data size
        data_bytes_size = num_points * num_fields * 4  # 4 bytes per float
        
        # Read the data portion directly from the current position
        data_bytes = self.shared_pc_data.read(data_bytes_size)
        
        # Convert to numpy array
        float_array = np.frombuffer(data_bytes, dtype=np.float32)
        
        # Reshape based on the dimensions
        reshaped_array = float_array.reshape(num_points, num_fields)
        
        # Convert to torch tensor
        tensor = torch.from_numpy(reshaped_array).cuda()
        self.mutex.release()
        
        return tensor


    def pc_callback(self, pc_ready_msg):
        if self.delegate_gt:
            if self.latest_gt_queue:
                # Convert all timestamps to seconds
                pc_time = pc_ready_msg.stamp.sec + pc_ready_msg.stamp.nanosec / 1e9
                timestamps = [s.header.stamp.sec + s.header.stamp.nanosec / 1e9 for s in self.latest_gt_queue]
                timestamps = np.array(timestamps)

                # Find all timestamps smaller than pc_time
                diffs = timestamps - pc_time
                smaller_indices = np.where(diffs <= 0)[0]

                if len(smaller_indices) > 0:
                    # Get the most recent one (largest timestamp that's still smaller than pc_time)
                    idx = smaller_indices[-1]
                    self.det_pub.publish(self.latest_gt_queue[idx])
                else:
                    self.get_logger().warn("No ground truth messages found before the point cloud timestamp")
            return

        if not self.model_initialized:
            return

        ridx = self.get_parameter('model_res_idx').value
        if ridx >= 0 and ridx < 6:
            self.model.res_idx = ridx

        #self.get_logger().info(f"Deadline is {round(self.model._default_deadline_sec * 1000)} ms")

        with record_function("inference"):
            model = self.model
            start_time = time.time()
            start_time_stamp = Time(nanoseconds=time.time_ns()).to_msg()
            start_time_mntc = time.monotonic()

            model.measure_time_start('End-to-end')
            model.measure_time_start('PreProcess')
            #deadline_sec_override, reset = model.initialize(sample_token)

            with torch.no_grad():
                #tensor = f32_multi_arr_to_tensor(multi_arr).cuda()
                tensor = self.read_from_shmem()
                # the reference frame of awsim is baselink, so we need to change that to
                # lidar by decreasing z
                batch_id = torch.zeros(tensor.size(0), dtype=tensor.dtype,  device=tensor.device)

                batch_dict = {
                    'points': torch.cat((batch_id.unsqueeze(-1), tensor), dim=1)
                }
                torch.cuda.synchronize() # remove me

                #self.get_logger().warn(f"Num points: {tensor.size(0)}")

                # Up to here, avege time spent is 3ms

                batch_dict['batch_size'] = 1
                batch_dict['scene_reset'] = False
                batch_dict['start_time_sec'] = start_time
                batch_dict['deadline_sec'] = self.model._default_deadline_sec
                batch_dict['abs_deadline_sec'] = start_time + batch_dict['deadline_sec']
                model.measure_time_end('PreProcess')
                batch_dict = model.forward(batch_dict)

                model.measure_time_start('PostProcess')
                if 'final_box_dicts' in  batch_dict:
                    if 'pred_ious' in batch_dict['final_box_dicts'][0]:
                        del batch_dict['final_box_dicts'][0]['pred_ious']
                    for k,v in batch_dict['final_box_dicts'][0].items():
                        batch_dict['final_box_dicts'][0][k] = v.cpu()

                model.latest_batch_dict = {k: batch_dict[k] \
                        for k in ['final_box_dicts']}

            model.measure_time_end('PostProcess')
            model.measure_time_end('End-to-end')

            torch.cuda.synchronize()
            model.calc_elapsed_times() 
            model.last_elapsed_time_musec = int(model._time_dict['End-to-end'][-1] * 1000)

            pred_dict = batch_dict['final_box_dicts'][0]
            pred_dict['pred_boxes'] = self.transform_to_base_link(pred_dict['pred_boxes'])

            end_time_mntc = time.monotonic()
            processing_time_ms = (end_time_mntc - start_time_mntc) * 1e3

        if model.simulate_exec_time:
            #assert not model.is_calibrating()
            num_points = batch_dict['points'].size(0)
            num_voxels = batch_dict.get('bb3d_num_voxels', None)
            if num_voxels is not None:
                num_voxels = np.array(num_voxels)
            xmin, xmax = batch_dict['tensor_slice_inds']
            xlen = xmax - xmin + 1
            exec_time_ms = model.calibrators[model.res_idx].pred_exec_time_ms(
                    num_points, num_voxels, xlen, consider_prep_time=True)
            #self.get_logger().info(f"Simulating resolution {model.res_idx}, exec time is {exec_time_ms} ms")

            # sleep to simulate it
            sleep_time_ms = exec_time_ms - processing_time_ms
            if sleep_time_ms > 0:
                time.sleep(sleep_time_ms * 1e-3)
                processing_time_ms = exec_time_ms
            else:
                self.get_logger().warn(f"Actual DNN execution time is higher than simulated time!")

        pc_stamp = pc_ready_msg.stamp
        self.publish_dets(pred_dict, pc_stamp, 'base_link')
        time_since_last_publish = time.monotonic() - self.publish_time_mntc
        self.publish_time_mntc = end_time_mntc

        int_msg = Int64Stamped()
        int_msg.stamp = start_time_stamp
        int_msg.data = int(self.model.res_idx)
        self.res_idx_pub.publish(int_msg)

        time_msg = Float64Stamped() 
        time_msg.stamp = start_time_stamp
        time_msg.data = processing_time_ms
        self.processing_time_pub.publish(time_msg)

        idx = self.sample_counter % len(self.past_publish_times) 
        self.past_publish_times[idx] = time_since_last_publish
        self.sample_counter += 1
        #if self.sample_counter % 10 == 0:
        #    print('VALOR publish latency:', np.mean(self.past_publish_times) * 1000, 'ms')

        if self.sample_counter % 100 == 0:
            model.print_time_stats()
            model.clear_stats()
            self.sample_counter = 0

    # This func takes less than 1 ms, ~0.6 ms
    def publish_dets(self, pred_dict, stamp, frame_id):
        # publish even if its empty
        #pred_dict = self.model.get_empty_det_dict()  # This is for debugging if objdet is necessary

        float_arr = pred_dict_to_f32_multi_arr(pred_dict, stamp, frame_id)
        self.arr_pub.publish(float_arr)
        if PUB_DEBUG_DETS:
            det_objs = f32_multi_arr_to_detected_objs(float_arr, self.cls_mapping)
            self.det_pub.publish(det_objs)

def RunInferenceNode(period_sec):
    rclpy.init(args=None)
    node = InferenceNode(period_sec)
    if PROFILE:
        with profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA]) as prof:
            try:
                rclpy.spin(node)
            except:
                pass
        prof.export_chrome_trace("trace.json")

    else:
        rclpy.spin(node)

    node.destroy_node()
    #rclpy.shutdown()

def main():
    #parser = argparse.ArgumentParser(description='arg parser')
    #parser.add_argument('--cfg_file', type=str, default=def_cfg_file, help='specify the config for eval')
    #parser.add_argument('--ckpt', type=str, default=def_ckpt_file, help='checkpoint to start from')
    #parser.add_argument('--set', dest='set_cfgs', default=None, nargs=argparse.REMAINDER,
    #                    help='set extra config keys if needed')

    #cmdline_args = parser.parse_args()
    period_sec = 0.05 # point cloud period
    #RunInferenceNode(cmdline_args, period_sec)
    RunInferenceNode(period_sec)

    print('InferenceNode finished execution.')

if __name__ == '__main__':
    main()

