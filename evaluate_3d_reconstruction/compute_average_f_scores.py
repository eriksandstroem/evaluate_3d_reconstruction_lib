# This script computes the average F-scores for the methods
# TSDF Fusion
# COLMAP
# Fusibile
# Bundlefusion
# RoutedFusion (not implemented yet)

# The average is taken over the scenes office_0, office_4 and hotel_0
# The result is printed in the terminal
import os
import numpy as np

base_dir = '/scratch_net/nudel_second/esandstroem/opportunistic_3d_capture/Erik_3D_Reconstruction_Project/3rdparty/evaluate_3d_reconstruction/data/replica'

sceneList = ['office_0', 'office_4', 'hotel_0']

data_dict = {'TSDF Fusion': dict(), 'COLMAP': dict(), 'Fusibile': dict(), 'BundleFusion': dict(), 'RoutedFusion': dict()}

name_dict =  {'fuse_grids_tof_stereo_from_indep_trained_models_artificial_trunc': 'fused_tof_psmnet', 'left_bts_depth_artificial_trunc': 'bts', 'left_psmnet_depth_artificial_trunc': 'psmnet', 'left_depth_noise_5.0_artificial_trunc': 'tof', 'left_depth_gt_artificial_trunc': 'gt'}

mode_list = ['fuse_grids_tof_stereo_from_indep_trained_models_artificial_trunc', 'gt_depth_artificial_trunc', 'mono_no_routing_artificial_trunc', 'multidepth_fusionNet_artificial_trunc', 
			'multidepth_fusionNet_conditioned_artificial_trunc', 'multidepth_routingNet_artificial_trunc', 'multidepth_three_fusionNet_artificial_trunc', 'multidepth_two_fusionNet_artificial_trunc',
			'stereo_artificial_trunc', 'stereo_no_routing_artificial_trunc', 'tof_artificial_trunc', 'tof_no_routing_artificial_trunc']
# TSDF Fusion
print('===========================')
print('TSDF Fusion')
for scene in sceneList:
	for mode in os.listdir(base_dir + '/' + scene + '/tsdf_fusion'):
		data_path = base_dir + '/' + scene + '/tsdf_fusion/' + mode + '/evaluation/' + scene + '.prf_tau_plotstr.txt'
		data = np.loadtxt(data_path)
		if mode in data_dict['TSDF Fusion'].keys():
			data_dict['TSDF Fusion'][mode] += data[0:3]
		else:
			data_dict['TSDF Fusion'][mode] = data[0:3]

# Average the results over the scenes
print('---------------------------')
for key in data_dict['TSDF Fusion'].keys():
	data_dict['TSDF Fusion'][key] /= len(sceneList)
	print(name_dict[key])
	print('Precision: ', data_dict['TSDF Fusion'][key][0])
	print('Recall: ', data_dict['TSDF Fusion'][key][1])
	print('F-score: ', data_dict['TSDF Fusion'][key][2])
	print('---------------------------')
	

# COLMAP
print('===========================')
print('COLMAP')
for scene in sceneList:
	for mode in os.listdir(base_dir + '/' + scene + '/colmap'):
		data_path = base_dir + '/' + scene + '/colmap/' + mode + '/evaluation/' + scene + '.prf_tau_plotstr.txt'
		data = np.loadtxt(data_path)
		if 'pointcloud' in data_dict['COLMAP'].keys():
			data_dict['COLMAP']['pointcloud'] += data[0:3]
		else:
			data_dict['COLMAP']['pointcloud'] = data[0:3]


# Average the results over the scenes
print('---------------------------')
for key in data_dict['COLMAP'].keys():
	data_dict['COLMAP'][key] /= len(sceneList)
	print(key)
	print('Precision: ', data_dict['COLMAP'][key][0])
	print('Recall: ', data_dict['COLMAP'][key][1])
	print('F-score: ', data_dict['COLMAP'][key][2])
	print('---------------------------')

# Fusibile
print('===========================')
print('Fusibile')
for scene in sceneList:
	for mode in os.listdir(base_dir + '/' + scene + '/fusibile'):
		data_path = base_dir + '/' + scene + '/fusibile/' + mode + '/evaluation/' + scene + '.prf_tau_plotstr.txt'
		data = np.loadtxt(data_path)
		if mode.split('_')[2] in data_dict['Fusibile'].keys():
			data_dict['Fusibile'][mode.split('_')[2]] += data[0:3]
		else:
			data_dict['Fusibile'][mode.split('_')[2]] = data[0:3]


# print('---------------------------')
for key in data_dict['Fusibile'].keys():
	data_dict['Fusibile'][key] /= len(sceneList)
	print(key)
	print('Precision: ', data_dict['Fusibile'][key][0])
	print('Recall: ', data_dict['Fusibile'][key][1])
	print('F-score: ', data_dict['Fusibile'][key][2])
	print('---------------------------')

# BundleFusion
print('===========================')
print('BundleFusion')
for scene in sceneList:
	for mode in os.listdir(base_dir + '/' + scene + '/bundlefusion'):
		data_path = base_dir + '/' + scene + '/bundlefusion/' + mode + '/evaluation/' + scene + '.prf_tau_plotstr.txt'
		data = np.loadtxt(data_path)
		if mode.split('_')[2] in data_dict['BundleFusion'].keys():
			data_dict['BundleFusion'][mode.split('_')[2]] += data[0:3]
		else:
			data_dict['BundleFusion'][mode.split('_')[2]] = data[0:3]


print('---------------------------')
for key in data_dict['BundleFusion'].keys():
	data_dict['BundleFusion'][key] /= len(sceneList)
	print(key)
	print('Precision: ', data_dict['BundleFusion'][key][0])
	print('Recall: ', data_dict['BundleFusion'][key][1])
	print('F-score: ', data_dict['BundleFusion'][key][2])
	print('---------------------------')


# RoutedFusion
print('===========================')
print('RoutedFusion')
for scene in sceneList:
	for mode in os.listdir(base_dir + '/' + scene + '/routedfusion'):
		if mode in mode_list:
			data_path = base_dir + '/' + scene + '/routedfusion/' + mode + '/evaluation/' + scene + '.prf_tau_plotstr.txt'
			data = np.loadtxt(data_path)
			mode_name = '_'.join(mode.split('_')[:-2])
			if mode_name in data_dict['RoutedFusion'].keys():
				data_dict['RoutedFusion'][mode_name] += data[0:3]
			else:
				data_dict['RoutedFusion'][mode_name] = data[0:3]


print('---------------------------')
for key in data_dict['RoutedFusion'].keys():
	data_dict['RoutedFusion'][key] /= len(sceneList)
	print(key)
	print('Precision: ', data_dict['RoutedFusion'][key][0])
	print('Recall: ', data_dict['RoutedFusion'][key][1])
	print('F-score: ', data_dict['RoutedFusion'][key][2])
	print('---------------------------')


# compute average IOU, L1, MSE, Acc for TSDF Fusion and RoutedFusion below