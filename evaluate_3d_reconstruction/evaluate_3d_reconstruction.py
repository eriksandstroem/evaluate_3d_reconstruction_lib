#!/cluster/project/cvl/esandstroem/virtual_envs/multisensor_env_python_gpu_3.8.5/bin/python
import os

# ----------------------------------------------------------------------------
# -                   TanksAndTemples Website Toolbox                        -
# -                    http://www.tanksandtemples.org                        -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2017
# Arno Knapitsch <arno.knapitsch@gmail.com >
# Jaesik Park <syncle@gmail.com>
# Qian-Yi Zhou <Qianyi.Zhou@gmail.com>
# Vladlen Koltun <vkoltun@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ----------------------------------------------------------------------------
#
# This python script is for downloading dataset from www.tanksandtemples.org
# The dataset has a different license, please refer to
# https://tanksandtemples.org/license/

# this script requires Open3D python binding
# please follow the intructions in setup.py before running this script.

# This is script is modified by Erik Sandstroem

# Run this script in the folder of the target .ply file by providing the name of the .ply file as
# the input. The evaluation output is stored in the corresponding folder located in the data folder
# of this script.

# example use case: go to the directory where the reconstructed .ply file is located
# then run evaluate_3d_reconstruction.py somename.ply

import numpy as np
import open3d as o3d
from sys import argv
import pathlib

from evaluate_3d_reconstruction.config import *
from evaluate_3d_reconstruction.evaluation import EvaluateHisto
from evaluate_3d_reconstruction.util import make_dir
from evaluate_3d_reconstruction.plot import plot_graph

def run_evaluation(pred_ply, ground_truth_data, scene, path_to_pred_ply, transformation=None):
    if transformation:
        gt_trans = np.loadtxt(base_transformation_dir + '/' + scene + '/' + transformation)
    else:
        gt_trans = np.eye(4)

    if ground_truth_data == 'watertight':
        gt_ply_path = watertight_ground_truth_data_base + '/' + scene + '_processed.ply' # ground truth .ply file
    elif ground_truth_data == 'standard_trunc':
        gt_ply_path = standard_trunc_ground_truth_data_base + '/' + scene + '.ply' 
    elif ground_truth_data == 'artificial_trunc':
        gt_ply_path = artificial_trunc_ground_truth_data_base + '/' + scene + '.ply' 

    pred_ply_path = path_to_pred_ply + '/' + pred_ply

    out_dir = path_to_pred_ply + '/' + pred_ply[:-4]

    make_dir(out_dir)

    print("")
    print("===========================")
    print("Evaluating %s" % scene)
    print("===========================")

    dTau = 0.02 # constant Tau regardless of scene size

    # Load reconstruction and according GT
    # Use these four lines below to also load the normals from the mesh. Note
    # the implementation is not complete - I turned off the normal estimation
    # step in evaluation.py because this caused artifacts. Instead
    # what I need to do is to make sure that the normals which I load here
    # are kept thorugh the cropping and downsampling function of the point cloud.
    # mesh = trimesh.load(pred_ply_path)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    # pcd.normals = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_normals))
    pcd = o3d.io.read_point_cloud(pred_ply_path) # samples the vertex coordinate points of the mesh
    mesh = o3d.io.read_triangle_mesh(pred_ply_path) # mesh which we want to color for precision
    # print('pcd: ', np.asarray(pcd.points).shape)
    # m = o3d.io.read_triangle_mesh(pred_ply_path)
    # print('m: ', np.asarray(m.vertices).shape)

    gt_pcd = o3d.io.read_point_cloud(gt_ply_path)
    gt_mesh = o3d.io.read_triangle_mesh(gt_ply_path) # mesh to color for recall

    dist_threshold = dTau
    voxel_size = 0.01

    # Histogramms and P/R/F1
    plot_stretch = 5
    [
        precision,
        recall,
        fscore,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
        min1,
        min2,
        max1,
        max2,
        mean1,
        mean2,
        median1,
        median2,
        std1,
        std2,
    ] = EvaluateHisto(
        pcd,
        mesh,
        gt_pcd,
        gt_mesh,
        gt_trans,
        voxel_size,
        dTau,
        out_dir,
        plot_stretch,
        scene,
    )
    eva = [precision, recall, fscore]
    print("==============================")
    print("evaluation result : %s" % scene)
    print("==============================")
    print("distance tau : %.3f" % dTau)
    print("precision : %.4f" % eva[0])
    print("recall : %.4f" % eva[1])
    print("f-score : %.4f" % eva[2])
    print("==============================")
    print("precision statistics")
    print("min: %.4f" % min1)
    print("max: %.4f" % max1)
    print("mean: %.4f" % mean1)
    print("median: %.4f" % median1)
    print("std: %.4f" % std1)
    print("==============================")
    print("recall statistics")
    print("min: %.4f" % min2)
    print("max: %.4f" % max2)
    print("mean: %.4f" % mean2)
    print("median: %.4f" % median2)
    print("std: %.4f" % std2)
    print("==============================")


    # Plotting
    plot_graph(
        scene,
        fscore,
        dist_threshold,
        edges_source,
        cum_source,
        edges_target,
        cum_target,
        plot_stretch,
        out_dir,
    )


if __name__ == "__main__":


    pred_ply = argv[1] # name of predicted .ply file
    ground_truth_data = argv[2] # watertight or artificial_trunc or standard_trunc 
    scene = argv[3] # scene
    if len(argv) == 5:
        transformation= argv[4] # open3d or lewiner
    else:
        transformation=None

    run_evaluation(
        pred_ply=pred_ply,
        ground_truth_data=ground_truth_data,
        scene=scene,
        path_to_pred_ply=str(pathlib.Path().absolute()),
        transformation=transformation

    )
