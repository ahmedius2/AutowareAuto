import argparse
import datetime
import os
import time
import json
import queue
import torch
import math
import numpy as np
from pathlib import Path

from pcdet.utils.ros2_utils import get_dataset, pred_dict_to_f32_multi_arr, f32_multi_arr_to_detected_objs
from pcdet.config import cfg, cfg_from_list, cfg_from_yaml_file, log_config_to_file
from pcdet.models import build_network 

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.clock import ClockType, Clock
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header, MultiArrayDimension
from autoware_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification
from autoware_vehicle_msgs.msg import VelocityReport
from valo_msgs.msg import Float32MultiArrayStamped
from torch.profiler import profile, record_function, ProfilerActivity
from tier4_debug_msgs.msg import Float64Stamped
#from callback_profile.msg import CallbackProfile

PROFILE = False
PUB_DEBUG_DETS = False
DYN_RES = False

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

        self.period_sec = period_sec
        self.arr_pub = self.create_publisher(Float32MultiArrayStamped,
                'valor_detected_objs', 10)
        self.processing_time_pub = self.create_publisher(Float64Stamped,
                '~/execution_time_ms', 10)
        if PUB_DEBUG_DETS:
            self.det_pub = self.create_publisher(DetectedObjects,
                    '~/debug/valor_detected_obj', 10)
        self.pc_sub = self.create_subscription(Float32MultiArrayStamped,
                'accumulated_pc_as_arr', self.pc_callback, 1)
        self.velo_sub = self.create_subscription(VelocityReport,
                '/vehicle/status/velocity_status', self.vel_callback, 1)

        self.model_initialized = False
        self.init_model()
        self.model_initialized = True

        self.past_publish_times = np.zeros(10)
        self.vel_limit = 10 # m/s
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

    def tranform_to_base_link(self, objects):
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

        def_cfg_file  = "tools/cfgs/nuscenes_models/mural_pillarnet_0100_0128_0200_awsim.yaml"
        def_cfg_file  = os.path.join(pth, def_cfg_file)
        def_ckpt_file = "models/mural_pillarnet_0100_0128_0200_awsim_e20.pth"
        def_ckpt_file = os.path.join(pth, def_ckpt_file)

        cfg_from_yaml_file(def_cfg_file, cfg)
        #if args.set_cfgs is not None:
        set_cfgs = ['MODEL.METHOD', 12,
            'MODEL.DEADLINE_SEC', 0.100,
            'MODEL.DENSE_HEAD.NAME', 'CenterHeadInf',
            'OPTIMIZATION.BATCH_SIZE_PER_GPU', '1']
        cfg_from_list(set_cfgs, cfg)

        logger, test_set = get_dataset(cfg)

        model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=test_set)
        model.load_params_from_file(filename=def_ckpt_file, logger=logger, to_cpu=False)
        model.eval()
        model.cuda()
        self.model_cfg = cfg

        self.dataset = model.dataset

        oc = ObjectClassification()
        cls_names = [str(i) for i in cfg.CLASS_NAMES]
        self.cls_mapping = { cls_names.index(name)+1: oc.__getattribute__(name.upper()) \
                for name in cls_names }

        #for k, m in self.cls_mapping.items():
        #    self.get_logger().warn(str(k) + ":" + str(m))

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
        if DYN_RES:
            longtd_vel = vel_report.longitudinal_velocity
            lateral_vel = vel_report.lateral_velocity

            vel = math.sqrt(longtd_vel**2 + lateral_vel**2)

            num_res = 5
            self.model.res_idx = round((vel / self.vel_limit) * num_res)
            if self.model.res_idx >= num_res:
                self.model.res_idx = num_res - 1

    def pc_callback(self, multi_arr):
        if not self.model_initialized:
            return

        ridx = self.get_parameter('model_res_idx').value
        if ridx >= 0 and ridx < 5:
            self.model.res_idx = ridx

        with record_function("inference"):
            model = self.model
            start_time = time.time()
            start_time_stamp = Time(nanoseconds=time.time_ns()).to_msg()
            start_time_mntc = time.monotonic()

            model.measure_time_start('End-to-end')
            model.measure_time_start('PreProcess')
            #deadline_sec_override, reset = model.initialize(sample_token)

            with torch.no_grad():
                tensor = f32_multi_arr_to_tensor(multi_arr).cuda()
                # the reference frame of awsim is baselink, so we need to change that to
                # lidar by decreasing z
                batch_id = torch.zeros(tensor.size(0), dtype=tensor.dtype,  device=tensor.device)

                batch_dict = {
                    'points': torch.cat((batch_id.unsqueeze(-1), tensor), dim=1)
                }
                torch.cuda.synchronize() # remove me

                # Up to here, avege time spent is 3ms

                batch_dict['batch_size'] = 1
                batch_dict['scene_reset'] = False
                batch_dict['start_time_sec'] = start_time
                batch_dict['deadline_sec'] = 10.0
                batch_dict['abs_deadline_sec'] = start_time + batch_dict['deadline_sec']
                model.measure_time_end('PreProcess')
                batch_dict = model.forward(batch_dict)

                model.measure_time_start('PostProcess')
                if 'final_box_dicts' in  batch_dict:
                    if 'pred_ious' in batch_dict['final_box_dicts'][0]:
                        del batch_dict['final_box_dicts'][0]['pred_ious']
                    for k,v in batch_dict['final_box_dicts'][0].items():
                        batch_dict['final_box_dicts'][0][k] = v.cpu()

            model.latest_batch_dict = {k: batch_dict[k] for k in \
                    ['final_box_dicts']}

            model.measure_time_end('PostProcess')
            model.measure_time_end('End-to-end')

            torch.cuda.synchronize()
            model.calc_elapsed_times() 
            model.last_elapsed_time_musec = int(model._time_dict['End-to-end'][-1] * 1000)

            pred_dict = batch_dict['final_box_dicts'][0]
            pred_dict['pred_boxes'] = self.tranform_to_base_link(pred_dict['pred_boxes'])

            pc_stamp = multi_arr.header.stamp
            self.publish_dets(pred_dict, pc_stamp)
            end_time_mntc = time.monotonic()
            time_since_last_publish = end_time_mntc - self.publish_time_mntc
            self.publish_time_mntc = end_time_mntc

            processing_time_ms = (end_time_mntc - start_time_mntc) * 1e3
            time_msg = Float64Stamped() 
            time_msg.stamp = start_time_stamp
            time_msg.data = processing_time_ms
            self.processing_time_pub.publish(time_msg)

            # NOTE The following appears to be not working well
            #cur_time = self.get_clock().now()
            #pipeline_time = cur_time - Time.from_msg(pc_stamp)
            #time_msg.data = pipeline_time.nanoseconds * 1e-6
            #self.pipeline_time_pub.publish(time_msg)

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
    def publish_dets(self, pred_dict, stamp):
        # publish even if its empty
        float_arr = pred_dict_to_f32_multi_arr(pred_dict, stamp)
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
    period_sec = 0.1 # point cloud period
    #RunInferenceNode(cmdline_args, period_sec)
    RunInferenceNode(period_sec)

    print('InferenceNode finished execution.')

if __name__ == '__main__':
    main()

