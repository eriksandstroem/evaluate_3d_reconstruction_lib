#!/cluster/project/cvl/esandstroem/virtual_envs/multisensor_env_python_gpu_3.8.5/bin/python

# This script is modified from the original source by Erik Sandstroem

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

# See README.md file for usage instructions

import numpy as np
import open3d as o3d
from sys import argv
import pathlib

from evaluate_3d_reconstruction.config import (
    ground_truth_data_base,
    base_transformation_dir,
)
from evaluate_3d_reconstruction.evaluation import EvaluateHisto
from evaluate_3d_reconstruction.util import make_dir
from evaluate_3d_reconstruction.plot import plot_graph


def run_evaluation(pred_ply, path_to_pred_ply, scene, transformation=None):
    """Calculates the F-score from a predicted mesh to a reference mesh. Generates
    a directory and fills this numerical and mesh results.

        Args:
            pred_ply: string object to denote the name of predicted mesh (as a .ply file)
            scene: string object to denote the scene name (a corresponding ground
                        truth .ply file with the name "scene + .ply" needs to exist)
            path_to_pred_ply: string object to denote the full path to the pred_ply file
            transformation: boolean to denote if to use the available transformation matrix for
                        the ground truth mesh.

        Returns:
            None"""

    # load transformation matrix
    if transformation:
        gt_trans = np.loadtxt(
            base_transformation_dir + "/" + scene + "/" + transformation
        )
    else:
        gt_trans = np.eye(4)

    # specify path to ground truth mesh
    gt_ply_path = ground_truth_data_base + "/" + scene + ".ply"

    # full path to predicted mesh
    pred_ply_path = path_to_pred_ply + "/" + pred_ply

    # output directory
    out_dir = path_to_pred_ply + "/" + pred_ply[:-4]

    # create output directory
    make_dir(out_dir)

    print("")
    print("===========================")
    print("Evaluating %s" % scene)
    print("===========================")

    dTau = 0.02  # constant Tau regardless of scene size

    # Load reconstruction and corresponding GT
    pcd = o3d.io.read_point_cloud(
        pred_ply_path
    )  # samples the vertex coordinate points of the mesh
    mesh = o3d.io.read_triangle_mesh(
        pred_ply_path
    )  # mesh which we want to color for precision

    gt_pcd = o3d.io.read_point_cloud(gt_ply_path)
    gt_mesh = o3d.io.read_triangle_mesh(
        gt_ply_path
    )  # mesh which we want to color for recall

    dist_threshold = dTau
    voxel_size = 0.01

    # Histograms and P/R/F
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

    pred_ply = argv[1]  # name of predicted .ply file
    scene = argv[2]  # scene name
    if len(argv) == 4:
        transformation = argv[
            3
        ]  # transformation matrix to be applied if the meshes are not aligned
    else:
        transformation = None

    run_evaluation(
        pred_ply=pred_ply,
        path_to_pred_ply=str(pathlib.Path().absolute()),
        scene=scene,
        transformation=transformation,
    )
