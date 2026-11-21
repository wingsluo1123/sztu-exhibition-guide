#!/usr/bin/env python3
"""Measured replay coverage and consistency, not surveyed pose accuracy."""
import argparse
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--name', default='localization01')
    args = parser.parse_args(); evidence = ROOT/'evidence'/args.name
    output = ROOT/'maps'/args.name
    odom = np.loadtxt(output/'odom.csv', delimiter=',', skiprows=1)
    poses = np.loadtxt(output/'localization.csv', delimiter=',', skiprows=1)
    # on_activate publishes the supplied initial pose before any scan; omit it.
    poses = poses[(poses[:,0] >= odom[0,0]) & (poses[:,0] <= odom[-1,0])]
    poses = poses[np.r_[True, np.diff(poses[:,0])>0]]
    t = poses[:,0]
    local_positions = np.column_stack([np.interp(t, odom[:,0], odom[:,i]) for i in [1,2,3]])
    local_rotations = Slerp(odom[:,0], Rotation.from_quat(odom[:,4:8]))(t).as_matrix()
    map_rotations = Rotation.from_quat(poses[:,4:8]).as_matrix()
    alignment = map_rotations[0] @ local_rotations[0].T
    aligned = (local_positions-local_positions[0]) @ alignment.T + poses[0,1:4]
    difference = np.linalg.norm(aligned[:,:2]-poses[:,1:3], axis=1)
    gaps = np.diff(t)
    jumps = np.linalg.norm(np.diff(poses[:,1:3], axis=0), axis=1)
    moving = np.r_[False, np.linalg.norm(np.diff(local_positions[:,:2],axis=0),axis=1)/gaps > 0.05]
    correction_rotations = map_rotations @ local_rotations.transpose(0,2,1)
    correction_positions = poses[:,1:4] - np.einsum('nij,nj->ni',correction_rotations,local_positions)
    correction_steps = np.linalg.norm(np.diff(correction_positions,axis=0),axis=1)
    correction_angles = Rotation.from_matrix(
        correction_rotations[:-1].transpose(0,2,1) @ correction_rotations[1:]).magnitude()
    # Keep first-scan metrics above, and separately compare after startup settles.
    # The fixed 2..5 s window is checked for stationarity, not fitted to the route.
    window = (t >= t[0]+2.) & (t <= t[0]+5.)
    static_extent = np.ptp(local_positions[window],axis=0)
    static_rotation = Rotation.from_matrix(
        local_rotations[window][0].T @ local_rotations[window]).magnitude().max()
    settled = None
    if np.linalg.norm(static_extent)<0.05 and static_rotation<0.05:
        reference_rotation = Rotation.from_matrix(correction_rotations[window]).mean().as_matrix()
        reference_translation = np.median(correction_positions[window],axis=0)
        settled = local_positions @ reference_rotation.T + reference_translation
    settled_difference = np.linalg.norm(settled[:,:2]-poses[:,1:3],axis=1) if settled is not None else None
    post = t >= t[0]+5.
    logfile = evidence/'localizer.log'
    if not logfile.exists():
        logfile = evidence/'bringup.log'
    text = logfile.read_text()
    fitness = np.array([float(x) for x in re.findall(r'fitness score: ([\d.eE+\-]+)', text)])
    timings = np.array([float(x) for x in re.findall(r'align time:([\d.eE+\-]+)', text)])
    report = dict(no_surveyed_ground_truth=True, samples=len(t),
                  temporal_span_sec=float(t[-1]-t[0]),
                  gap_sec_percentiles=np.percentile(gaps,[50,95,99,100]).tolist(),
                  scan_step_xy_m_percentiles=np.percentile(jumps,[50,95,99,100]).tolist(),
                  ndt_fitness_percentiles=np.percentile(fitness,[50,95,99,100]).tolist(),
                  registration_wall_sec_percentiles=np.percentile(timings,[50,95,99,100]).tolist(),
                  start_aligned_lio_xy_difference_m_percentiles=np.percentile(difference,[50,95,99,100]).tolist(),
                  moving_samples=int(moving.sum()),
                  moving_start_aligned_lio_xy_difference_m_percentiles=(np.percentile(difference[moving],[50,95,99,100]).tolist() if moving.any() else None),
                  map_odom_translation_step_m_percentiles=np.percentile(correction_steps,[50,95,99,100]).tolist(),
                  map_odom_rotation_step_rad_percentiles=np.percentile(correction_angles,[50,95,99,100]).tolist(),
                  stationary_alignment_window_sec=[2,5],
                  stationary_window_xyz_extent_m=static_extent.tolist(),
                  stationary_window_rotation_extent_rad=float(static_rotation),
                  settled_start_lio_xy_difference_m_percentiles=(np.percentile(settled_difference[post],[50,95,99,100]).tolist() if settled is not None else None),
                  post_startup_map_odom_translation_step_m_percentiles=np.percentile(correction_steps[post[1:]],[50,95,99,100]).tolist(),
                  post_startup_map_odom_rotation_step_rad_percentiles=np.percentile(correction_angles[post[1:]],[50,95,99,100]).tolist(),
                  start_end_xy_distance_m=float(np.linalg.norm(poses[-1,1:3]-poses[0,1:3])),
                  transform_warnings=text.count('Could not get transform'),
                  bad_fitness_rejections=text.count('fitness score is over'),
                  convergence_rejections=text.count("registration didn't converge"))
    (evidence/'analysis.json').write_text(json.dumps(report, indent=2))
    map_dir = ROOT/'maps/mapping02'; config = yaml.safe_load((map_dir/'map.yaml').read_text())
    grid = np.load(map_dir/'occupancy.npy'); res = config['resolution']; x,y = config['origin'][:2]
    image = np.where(grid<0, .6, np.where(grid==0,1.,0.))
    fig, ax = plt.subplots(figsize=(9,8))
    ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=1,
              extent=[x,x+grid.shape[1]*res,y,y+grid.shape[0]*res])
    ax.plot(poses[:,1],poses[:,2],label='NDT / independent recorded sensors',lw=1.5)
    plotted = settled if settled is not None else aligned
    ax.plot(plotted[:,0],plotted[:,1],'--',label='LIO / stationary-start alignment' if settled is not None else 'LIO / first-scan alignment',lw=1)
    ax.scatter(*poses[0,1:3],marker='o',s=40,label='first matched scan')
    ax.set(xlabel='map x (m)',ylabel='map y (m)',title='Recorded-data localization; no surveyed ground truth')
    points=np.vstack([poses[:,1:3],plotted[:,:2]])
    ax.set_xlim(points[:,0].min()-1.5,points[:,0].max()+1.5)
    ax.set_ylim(points[:,1].min()-1.5,points[:,1].max()+1.5)
    ax.set_aspect('equal');ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(evidence/'trajectory.png',dpi=160);plt.close(fig)
    print(json.dumps(report))


if __name__ == '__main__':
    main()
