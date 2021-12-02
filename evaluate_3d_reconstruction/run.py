# Legacy script - not used anymore
import os

path = "/home/esandstroem/scratch-second/opportunistic_3d_capture/Erik_3D_Reconstruction_Project/src/RoutedFusion/workspace/fusion/multidepth_routingNet/test/"
os.chdir(path)
for scene_id in os.listdir(path):
    if scene_id.startswith("mesh"):
        print(
            "running script: evaluate_3d_reconstruction.py "
            + scene_id
            + " watertight routedfusion"
        )
        os.system(
            "evaluate_3d_reconstruction.py " + scene_id + " watertight routedfusion"
        )
    # break


# path = '/home/esandstroem/scratch-second/opportunistic_3d_capture/Erik_3D_Reconstruction_Project/3rdparty/evaluate_3d_reconstruction/data/replica/office_0/routedfusion/tof_watertight/'
# # print the different precision scores
# prec_mat = np.zeros((5, 15))
# rec_mat = np.zeros((5, 15))
# f_mat = np.zeros((5, 15))
# for scene_id in os.listdir(path):
# 	# print(scene_id[-1])
# 	row = int(scene_id[-1])
# 	# print(row)
# 	col = int(scene_id[-4])
# 	col2 = scene_id[-5]
# 	if col2.isdigit():
# 		col = int(scene_id[-5:-3])
# 	# print(col)
# 	file = path + scene_id + '/office_0.prf_tau_plotstr.txt'
# 	# open file
# 	nbrs = np.loadtxt(file)
# 	prec_mat[row, col] = np.round(nbrs[0], 4)
# 	rec_mat[row, col] = np.round(nbrs[1], 4)
# 	f_mat[row, col] = np.round(nbrs[2], 4)

# print('Precision Scores, Stereo Column wise, ToF Row wise')
# print(prec_mat)
# print('Recall Scores, Stereo Column wise, ToF Row wise')
# print(rec_mat)
# print('F-Score Scores, Stereo Column wise, ToF Row wise')
# print(f_mat)
