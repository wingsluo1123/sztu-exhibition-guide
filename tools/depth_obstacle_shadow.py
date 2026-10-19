#!/usr/bin/env python3
"""Bounded depth/LiDAR shadow observations on the sensor host; no control outputs.

This diagnostic never publishes scans, TF, velocity or coverage flags. Point/cell
presence and single-pixel ray comparisons are observations, not free-space proof.
"""

import argparse
from collections import Counter, deque
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

import analyze_depth_lidar_pairs as geometry


def image_array(image):
    """Validate the 16UC1 layout, including padded rows and byte order."""
    if (image.encoding != '16UC1' or image.width <= 0 or image.height <= 0
            or image.step < image.width * 2 or image.step % 2
            or len(image.data) != image.step * image.height):
        raise ValueError('unsupported or truncated 16UC1 image layout')
    return np.ndarray((image.height, image.width),
                      dtype='>u2' if image.is_bigendian else '<u2',
                      buffer=image.data, strides=(image.step, 2))


def validate_camera(image, info, profile):
    """Reject calibration/frame changes rather than silently reusing old K."""
    k = geometry.intrinsics(info.k)
    expected = geometry.intrinsics(profile['k'])
    p = np.column_stack((k, np.zeros(3)))
    roi = info.roi
    if (image.header.frame_id != profile['frame']
            or info.header.frame_id != image.header.frame_id
            or (image.width, image.height) != (profile['width'], profile['height'])
            or (info.width, info.height) != (image.width, image.height)
            or not np.allclose(k, expected, atol=1e-6, rtol=0)
            or info.distortion_model != 'plumb_bob'
            or len(info.d) not in (5, 8) or not np.all(np.isfinite(info.d))
            or np.any(np.asarray(info.d) != 0)
            or not np.allclose(np.asarray(info.r).reshape(3, 3), np.eye(3), atol=1e-6, rtol=0)
            or not np.allclose(np.asarray(info.p).reshape(3, 4), p, atol=1e-6, rtol=0)
            or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
            or any((roi.x_offset, roi.y_offset, roi.width, roi.height, roi.do_rectify))):
        raise ValueError('camera calibration/frame/profile mismatch')
    return k


def ray_states(targets, depth, k, rotation, translation, scale, tolerance=.05):
    """Compare target axial depth with one rounded pixel; never label 'free'.

    0.2--5m is a diagnostic range, not a certified camera operating envelope.
    The 5cm surface band is a comparison heuristic, not measured sensor error.
    """
    pixels, axial, inside = geometry.project(targets, k, rotation, translation, depth.shape)
    labels = np.full(len(targets), 'outside_image', dtype='<U32')
    indices = np.flatnonzero(inside)
    uv = np.floor(pixels[inside] + .5).astype(int)
    bounded = (uv[:, 0] < depth.shape[1]) & (uv[:, 1] < depth.shape[0])
    indices, uv = indices[bounded], uv[bounded]
    observed = depth[uv[:, 1], uv[:, 0]].astype(float) * scale
    labels[indices] = 'invalid_depth'
    positive = np.isfinite(observed) & (observed > 0)
    labels[indices[positive]] = 'outside_diagnostic_range'
    valid = positive & (observed >= .2) & (observed <= 5)
    idx, delta = indices[valid], observed[valid] - axial[indices[valid]]
    labels[idx] = 'surface_band'
    labels[idx[delta < -tolerance]] = 'occluded'
    labels[idx[delta > tolerance]] = 'ray_reaches_target'
    return dict(Counter(labels.tolist()))


def summarize(depth, k, rotation, translation, lidar, normal, offset, scale):
    """Compare surface returns in the same depth-time leveled pelvis frame."""
    # Validate rigid geometry even when the depth image contains no valid pixels.
    geometry.project([], k, rotation, translation, depth.shape)
    if (not math.isfinite(scale) or scale <= 0 or np.asarray(normal).shape != (3,)
            or not np.all(np.isfinite(normal)) or not math.isfinite(offset)
            or not np.isclose(np.linalg.norm(normal), 1) or normal[2] <= 0):
        raise ValueError('positive scale and unit upward ground plane required')
    camera = geometry.depth_points(depth, k, scale, stride=4)
    camera = camera[(camera[:, 2] >= .2) & (camera[:, 2] <= 5)]
    points = camera @ rotation.T + translation
    heights = points @ normal - offset

    def surface_cells(pts):
        height = pts @ normal - offset
        keep = ((height >= .05) & (height <= 1.8) & (pts[:, 0] >= 0)
                & (pts[:, 0] <= 1.4) & (np.abs(pts[:, 1]) <= .55))
        selected = pts[keep]
        cells = {tuple(row) for row in np.floor(selected[:, :2] / .05).astype(int)}
        return selected, cells

    selected, depth_cells = surface_cells(points)
    _, lidar_cells = surface_cells(np.asarray(lidar).reshape(-1, 3))
    xy = geometry.swept_grid(.55, .85, .025)
    targets = []
    for height in (.05, .2, .6, 1.):
        z = (offset + height - xy @ normal[:2]) / normal[2]
        targets.append(dict(height_above_plane_m=height, total=len(xy),
                            states=ray_states(np.column_stack((xy, z)), depth, k,
                                              rotation, translation, scale)))
    depth_only = sorted(depth_cells - lidar_cells)
    result = dict(sampled_depth_points=len(points),
                  fraction_below_plane_5cm=float(np.mean(heights < -.05)) if len(points) else None,
                  near_roi_depth_points=len(selected), near_roi_depth_cells=len(depth_cells),
                  near_roi_lidar_cells=len(lidar_cells),
                  depth_cells_without_same_cell_lidar_return=len(depth_only),
                  # Bounded examples; not a map, not evidence that LiDAR cannot see them.
                  depth_only_cell_examples=[[int(x), int(y)] for x, y in depth_only[:200]],
                  nearest_depth_surface_xy_m=float(np.linalg.norm(selected[:, :2], axis=1).min())
                  if len(selected) else None, target_ray_comparisons=targets)
    return result


def run(args, output):
    # Lazy imports keep layout/geometry tests independent of the ROS runtime.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from tf2_ros import Buffer, TransformListener
    from sztu_navigation.ground_plane import estimate_ground
    from sztu_navigation.nav_geometry import rotation_matrix, xyz_points, yaw_of
    from sztu_slam.message_validation import MessageFreshness, stamp_nanoseconds

    profile = json.loads(args.profile.read_text())
    scale = float(profile['depth_scale'])
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('profile requires an explicit positive depth scale')
    rclpy.init()
    node = rclpy.create_node('sztu_depth_obstacle_shadow')
    buffer = Buffer()
    listener = TransformListener(
        buffer, node, qos=QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
    queues = {key: deque(maxlen=6) for key in ('depth', 'cloud', 'info')}
    guards = {key: MessageFreshness(.5) for key in queues}
    counts, rows, errors, captures = Counter(), [], [], []
    topics = dict(depth='/aima/hal/sensor/rgbd_head_front/depth_image',
                  info='/aima/hal/sensor/rgbd_head_front/depth_camera_info', cloud='/lio/points')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def receive(message, key):
        counts[key + '_received'] += 1
        mono = time.monotonic()
        if not guards[key].accept(message.header.stamp, node.get_clock().now().nanoseconds, mono):
            counts[key + '_timestamp_rejected'] += 1
            return
        queues[key].append((message, mono, guards[key].source_age))

    subscriptions = [node.create_subscription(
        kind, topics[key], lambda message, key=key: receive(message, key), qos)
        for key, kind in [('depth', Image), ('cloud', PointCloud2), ('info', CameraInfo)]]

    def driver_check():
        expected = profile['driver_parameters']
        deadline = time.monotonic() + 3
        endpoints = []
        while time.monotonic() < deadline:
            endpoints = node.get_publishers_info_by_topic(topics['depth'])
            if (len(endpoints) == 1
                    and not endpoints[0].node_name.startswith('_NODE_')
                    and not endpoints[0].node_namespace.startswith('_NODE_')):
                break
            rclpy.spin_once(node, timeout_sec=.02)
        if (len(endpoints) != 1 or endpoints[0].node_name.startswith('_NODE_')
                or endpoints[0].node_namespace.startswith('_NODE_')):
            return dict(matches=False, reason='unique named depth publisher not discovered')
        endpoint = endpoints[0]
        owner = endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name
        client = node.create_client(GetParameters, owner + '/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2):
                return dict(matches=False, reason='driver parameter service unavailable', owner=owner)
            future = client.call_async(GetParameters.Request(names=list(expected)))
            deadline = time.monotonic() + 2
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            if not future.done() or future.exception() is not None:
                return dict(matches=False, reason='driver parameter query failed', owner=owner)
            values = future.result().values
            observed = {}
            fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
            for name, value in zip(expected, values):
                observed[name] = getattr(value, fields[value.type]) if value.type in fields else None
            matches = (bool(expected) and len(values) == len(expected)
                       and all(type(observed[name]) is type(wanted) and observed[name] == wanted
                               for name, wanted in expected.items()))
            return dict(owner=owner, observed=observed, matches=matches,
                        stamp_ns=node.get_clock().now().nanoseconds)
        finally:
            node.destroy_client(client)

    def age(entry):
        message, mono, source_age = entry
        clock_age = (node.get_clock().now().nanoseconds - stamp_nanoseconds(message.header.stamp)) / 1e9
        if clock_age < -.05:
            return float('inf')
        return max(clock_age, source_age + time.monotonic() - mono)

    def transform(target, source, message):
        value = buffer.lookup_transform(target, source, Time.from_msg(message.header.stamp))
        q, v = value.transform.rotation, value.transform.translation
        quat = (q.x, q.y, q.z, q.w)
        if not np.isclose(np.linalg.norm(quat), 1, atol=1e-3, rtol=0):
            raise ValueError('non-unit TF quaternion')
        vector = np.array([v.x, v.y, v.z])
        if not np.all(np.isfinite(vector)):
            raise ValueError('non-finite TF translation')
        return rotation_matrix(quat), vector, quat

    driver_checks = [driver_check()]
    started, next_check, last_capture = time.monotonic(), 0., -math.inf
    last_used = None
    try:
        while driver_checks[0]['matches'] and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.01)
            mono = time.monotonic()
            if mono < next_check:
                continue
            next_check = mono + .2  # at most 5 diagnostic pairs/s
            for queue in queues.values():
                while queue and age(queue[0]) > .5:
                    queue.popleft()
            if not all(queues.values()):
                counts['waiting_inputs'] += 1
                continue
            candidates = [(abs(stamp_nanoseconds(i[0].header.stamp)
                               - stamp_nanoseconds(c[0].header.stamp)), i, c)
                          for i in queues['depth'] for c in queues['cloud']]
            gap, image_entry, cloud_entry = min(candidates, key=lambda row: row[0])
            image, cloud = image_entry[0], cloud_entry[0]
            depth_ns = stamp_nanoseconds(image.header.stamp)
            if last_used is not None and depth_ns <= last_used:
                counts['waiting_new_pair'] += 1
                continue
            info_entry = min(queues['info'], key=lambda entry:
                             abs(stamp_nanoseconds(entry[0].header.stamp) - depth_ns))
            info = info_entry[0]
            info_gap = abs(stamp_nanoseconds(info.header.stamp) - depth_ns) / 1e9
            if gap > 50_000_000 or info_gap > .05:
                counts['pair_gap_rejected'] += 1
                continue
            entries = (image_entry, cloud_entry, info_entry)
            try:
                k = validate_camera(image, info, profile)
                depth = image_array(image)
                if (cloud.is_bigendian or cloud.height != 1 or cloud.width <= 0
                        or cloud.point_step < 12 or cloud.row_step != len(cloud.data)
                        or cloud.width * cloud.point_step != len(cloud.data)
                        or not cloud.header.frame_id
                        or not all(any(f.name == name and f.offset == pos and f.datatype == 7
                                       and f.count == 1 for f in cloud.fields)
                                   for name, pos in [('x', 0), ('y', 4), ('z', 8)])):
                    raise ValueError('unsupported XYZ cloud layout')
                _, base, quat = transform('odom', 'base_link', image)
                camera_rotation, camera, _ = transform('odom', image.header.frame_id, image)
                lidar_rotation, lidar_translation, _ = transform('odom', cloud.header.frame_id, cloud)
                yaw = yaw_of(quat)
                level = rotation_matrix((0, 0, math.sin(yaw / 2), math.cos(yaw / 2))).T
                ankles = np.array([(transform('odom', frame, image)[1] - base) @ level.T
                                   for frame in ('left_ankle_roll_link', 'right_ankle_roll_link')])
                points = xyz_points(cloud.data, cloud.point_step)
                points = points[np.linalg.norm(points, axis=1) > 1e-6]
                points = (points @ lidar_rotation.T + lidar_translation - base) @ level.T
                ground = estimate_ground(points, ankles)
                if ground is None:
                    counts['ground_rejected'] += 1
                    continue
                rotation, translation = level @ camera_rotation, (camera - base) @ level.T
                result = summarize(depth, k, rotation, translation, points,
                                   ground.normal, ground.offset, scale)
            except ValueError as error:
                counts['validation_rejected'] += 1
                if len(errors) < 8:
                    errors.append(str(error))
                continue
            except Exception as error:
                counts['tf_or_processing_rejected'] += 1
                if len(errors) < 8:
                    errors.append(str(error))
                continue
            ages = [age(entry) for entry in entries]
            if max(ages) > .5:
                counts['expired_after_processing'] += 1
                continue
            last_used = depth_ns
            counts['pairs_observed'] += 1
            result.update(depth_stamp_ns=depth_ns,
                          cloud_stamp_ns=stamp_nanoseconds(cloud.header.stamp),
                          depth_cloud_gap_s=gap / 1e9, depth_info_gap_s=info_gap,
                          input_ages_at_record_s=ages,
                          elapsed_s=time.monotonic() - started,
                          ground_residual_p95_m=ground.residual_p95)
            rows.append(result)
            if len(captures) < 3 and mono - last_capture >= 5:
                path = output.parent / ('pair_' + str(len(captures)) + '.npz')
                with path.open('xb') as stream:
                    np.savez_compressed(stream, depth=depth, k=k, camera_rotation=rotation,
                                        camera_translation=translation, lidar_points=points,
                                        normal=ground.normal, offset=ground.offset, ankles=ankles,
                                        depth_stamp_ns=depth_ns,
                                        cloud_stamp_ns=stamp_nanoseconds(cloud.header.stamp))
                captures.append(dict(name=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
                last_capture = mono
        driver_checks.append(driver_check())
        publishers = {key: [dict(name=p.node_name, namespace=p.node_namespace)
                            for p in node.get_publishers_info_by_topic(topic)]
                      for key, topic in topics.items()}
        return dict(scope='Shadow observations only; no control, safety scan, TF or coverage publications. '
                          'No free-space or complete body-sweep coverage claim. Camera scale/axes remain '
                          'the prior single-pose hypothesis; no self-body masking or motion compensation '
                          'for objects moving between sensor times.',
                    diagnostic_completed=True, observations_available=bool(rows),
                    hardware_coverage_verified=False,
                    driver_checks=driver_checks,
                    profile_stable_at_checks=all(check['matches'] for check in driver_checks),
                    duration_s=time.monotonic() - started, ros_distro=os.environ.get('ROS_DISTRO'),
                    counts=dict(counts), first_errors=errors, rows=rows, captures=captures,
                    profile=profile, publishers=publishers,
                    diagnostic_model=dict(radius_m=.55, advance_with_stopping_margin_m=.85,
                                          target_spacing_m=.025, surface_cell_size_m=.05,
                                          axial_range_m=[.2, 5], surface_band_m=.05, depth_stride=4,
                                          max_age_s=.5, max_pair_gap_s=.05),
                    source_sha256={path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                   for path in (Path(__file__), Path(geometry.__file__), args.profile,
                                                Path(estimate_ground.__code__.co_filename),
                                                Path(xyz_points.__code__.co_filename),
                                                Path(stamp_nanoseconds.__code__.co_filename))})
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='new, unused directory; captures raw depth locally')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 120:
        parser.error('duration must be 1--120 seconds')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    output = args.output_dir / 'observations.json'
    report = run(args, output)
    serialized = json.dumps(report, indent=2, allow_nan=False)
    with output.open('x') as stream:
        stream.write(serialized + '\n')
    print(json.dumps(dict(output=str(output), counts=report['counts'],
                          profile_stable_at_checks=report['profile_stable_at_checks'],
                          driver_checks=report['driver_checks'],
                          first_errors=report['first_errors']), ensure_ascii=False))
    if not report['observations_available'] or not report['profile_stable_at_checks']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
