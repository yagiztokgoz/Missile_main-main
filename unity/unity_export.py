import json
import math
import os

import config
from missile.kinematics import _mat3tr


UNITY_OUTDIR = os.path.join(config.OUTDIR, 'unity')


def _normalize(vec):
    mag = math.sqrt(sum(v * v for v in vec))
    if mag < 1e-9:
        return [0.0, 0.0, 1.0]
    return [v / mag for v in vec]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _ned_to_unity(vec):
    north, east, down = vec
    return [east, -down, north]


def _vec3_dict(vec, scale=1.0):
    return {
        'x': float(vec[0] * scale),
        'y': float(vec[1] * scale),
        'z': float(vec[2] * scale),
    }


def _missile_axes_from_log_row(row):
    psi = math.radians(row['psiblx'])
    tht = math.radians(row['thtblx'])
    phi = math.radians(row['phiblx'])

    tbl = _mat3tr(psi, tht, phi)
    tlb = tbl.T

    forward_ned = tlb[:, 0].tolist()
    up_ned = (-tlb[:, 2]).tolist()

    return _normalize(_ned_to_unity(forward_ned)), _normalize(_ned_to_unity(up_ned))


def _target_forward_unity(positions_ned, idx):
    if len(positions_ned) <= 1:
        return [0.0, 0.0, 1.0]

    if idx == 0:
        delta = [positions_ned[1][i] - positions_ned[0][i] for i in range(3)]
    else:
        delta = [positions_ned[idx][i] - positions_ned[idx - 1][i] for i in range(3)]

    return _normalize(_ned_to_unity(delta))


def _safe_up(forward):
    world_up = [0.0, 1.0, 0.0]
    right = _cross(world_up, forward)
    if math.sqrt(sum(v * v for v in right)) < 1e-6:
        world_up = [1.0, 0.0, 0.0]
        right = _cross(world_up, forward)
    up = _cross(forward, right)
    return _normalize(up)


def export_unity_json(log, scenario, result, cpa, tag=None, stride=1, position_scale=1.0):
    """Export a Unity-friendly playback JSON from the in-memory simulation log."""
    if not log:
        raise ValueError('cannot export an empty log')
    if stride < 1:
        raise ValueError('stride must be >= 1')

    sampled_log = log[::stride]
    tag = tag or f's{scenario}'

    target_positions_ned = [
        [row['stel1'], row['stel2'], -row['stel_hbe']]
        for row in sampled_log
    ]

    frames = []
    for idx, row in enumerate(sampled_log):
        missile_pos_ned = [row['sbel1'], row['sbel2'], -row['hbe']]
        target_pos_ned = target_positions_ned[idx]

        missile_forward, missile_up = _missile_axes_from_log_row(row)
        target_forward = _target_forward_unity(target_positions_ned, idx)
        target_up = _safe_up(target_forward)

        frames.append({
            'time': float(row['t']),
            'missile': {
                'position': _vec3_dict(_ned_to_unity(missile_pos_ned), scale=position_scale),
                'forward': _vec3_dict(missile_forward),
                'up': _vec3_dict(missile_up),
                'speed_mps': float(row['dvbe']),
                'euler_deg': {
                    'yaw': float(row['psiblx']),
                    'pitch': float(row['thtblx']),
                    'roll': float(row['phiblx']),
                },
            },
            'target': {
                'position': _vec3_dict(_ned_to_unity(target_pos_ned), scale=position_scale),
                'forward': _vec3_dict(target_forward),
                'up': _vec3_dict(target_up),
            },
            'metrics': {
                'range_to_target_m': float(row['dtbc']),
                'time_to_go_s': float(row['tgoc']),
                'mach': float(row['mach']),
                'normal_accel_g': float(row['anx']),
                'lateral_accel_g': float(row['ayx']),
            },
        })

    sample_dt = frames[1]['time'] - frames[0]['time'] if len(frames) > 1 else 0.0
    payload = {
        'metadata': {
            'format': 'missile-unity-playback-v1',
            'scenario': int(scenario),
            'result': result,
            'position_scale': float(position_scale),
            'sample_stride': int(stride),
            'sample_dt_s': float(sample_dt),
            'duration_s': float(frames[-1]['time']),
            'coordinate_mapping': {
                'simulation_frame': 'NED (north, east, down)',
                'unity_frame': 'x=east, y=up, z=north',
            },
            'cpa': {
                'min_dist_m': float(cpa['min_dist']),
                't_cpa_s': float(cpa['t_cpa']),
                'position_sim': {
                    'north_m': float(cpa['pos'][0]),
                    'east_m': float(cpa['pos'][1]),
                    'altitude_m': float(cpa['pos'][2]),
                },
            },
        },
        'frames': frames,
    }

    os.makedirs(UNITY_OUTDIR, exist_ok=True)
    path = os.path.join(UNITY_OUTDIR, f'{tag}.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)

    print(f"Unity JSON saved -> {path}  ({len(frames)} frames)")
    return path
