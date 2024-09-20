import copy
import logging
import pandas as pd
import numpy as np
import argparse
import matplotlib.pyplot as plt

from evo.core.trajectory import PoseTrajectory3D
from evo.tools import plot, file_interface, log
from evo.core import sync
from evo.core.metrics import PoseRelation, Unit
import evo.main_ape
import evo.main_rpe

def get_result_traj(result_file):
    result_df = pd.read_csv(result_file)

    timestamp = result_df.iloc[:, 0].to_numpy(dtype=float)
    xyz = result_df.iloc[:, 1:4].to_numpy(dtype=float)
    quat = result_df.iloc[:, 4:8].to_numpy(dtype=float) # the quats need to be in wxyz

    traj_result = PoseTrajectory3D(xyz, quat, timestamp)
    return traj_result

def main():
    logger = logging.getLogger("evo")
    log.configure_logging(verbose=True)

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="subcommand", required=True)
    
    euroc_parser = subparsers.add_parser("euroc")
    euroc_parser.add_argument("gt_file")
    euroc_parser.add_argument("result_file")

    recording_parser = subparsers.add_parser("recording")
    recording_parser.add_argument("result_file")

    args = parser.parse_args()

    traj_gt = None
    traj_result = None

    logger.info("Loading the trajectories...")
    if args.subcommand == "euroc":
        traj_gt = file_interface.read_euroc_csv_trajectory(args.gt_file)
        traj_result = get_result_traj(args.result_file)
    elif args.subcommand == "recording": # recordings currently don't have ground truth
        traj_result = get_result_traj(args.result_file)
    
    # If a ground truth is provided, calculate and plot APE and RPE. Else, just plot the result trajectory.
    if traj_gt and traj_result:
        logger.info("Syncing the reference and ground truth trajectories...")
        # max_diff = 0.01 #seconds
        # offset_2 = 0 #if the second trajectory is offset
        traj_gt, traj_result = sync.associate_trajectories(traj_gt, traj_result)

        logger.info("Evaluating Absolute Pose Error...")
        ape = evo.main_ape.ape(traj_gt, traj_result, est_name='traj', pose_relation=PoseRelation.translation_part, align=True, correct_scale=True)

        logger.info("Evaluating Relative Pose Error...")
        rpe = evo.main_rpe.rpe(traj_gt, traj_result, est_name='rot', pose_relation=PoseRelation.rotation_angle_deg, delta= 1.0, delta_unit=Unit.frames, align=True, correct_scale=True)
    
        # Aligning and scaling trajectory before plotting
        traj_result_aligned_scaled = copy.deepcopy(traj_result)
        traj_result_aligned_scaled.align(traj_gt, correct_scale=True)

        logger.info("Creating plots for APE...")

        fig1 = plt.figure(figsize=(8, 8))
        plot.error_array(fig1.gca(), ape.np_arrays["error_array"],
                        name=ape.info["label"], xlabel="index")
        
        fig = plt.figure(figsize=(8, 8))
        plot_mode = plot.PlotMode.xyz
        ax = plot.prepare_axis(fig, plot_mode)
        plot.traj(ax, plot_mode, traj_gt, '--', 'gray')
        plot.traj(ax, plot_mode, traj_result_aligned_scaled, '-', 'blue')
        # plot.draw_coordinate_axes(ax, traj_gt, plot_mode)
        # plot.draw_coordinate_axes(ax, traj_result, plot_mode)
        plot.traj_colormap(ax, traj_gt, ape.np_arrays["error_array"], plot_mode,
                    min_map=ape.stats["min"], max_map=ape.stats["max"])
        fig.axes.append(ax)

        plot_collection = plot.PlotCollection(ape.info["title"])
        plot_collection.add_figure("raw", fig1)
        plot_collection.add_figure("map", fig)

        plot_collection.show()
        plot_collection.close()
    
    elif traj_result:
        logger.info("Plotting the result trajectory...")
       
        fig = plt.figure(figsize=(8, 8))
        plot_mode = plot.PlotMode.xyz
        ax = plot.prepare_axis(fig, plot_mode)
        plot.traj(ax, plot_mode, traj_result, '-', 'blue')
        # plot.draw_coordinate_axes(ax, traj_result, plot_mode)
        fig.axes.append(ax)

        plot_collection = plot.PlotCollection("MORB_SLAM result trajectory")
        plot_collection.add_figure("map", fig)

        plot_collection.show()
        plot_collection.close()


if __name__ == "__main__":
    main()