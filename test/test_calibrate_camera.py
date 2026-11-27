"""calibrate_camera.py: the camera mount from the floor plane in the depth image."""

import math

from conftest import load_script
import numpy as np
import pytest

# D435 depth at 640x480.
K = [383.0, 0.0, 320.0, 0.0, 383.0, 240.0, 0.0, 0.0, 1.0]
# Optical (x right, y down, z ahead) in a screw frame (x ahead, y left, z up).
OPT_IN_SCREW = np.array(
    [[0.0, 0.0, 1.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)
OPT_IN_SCREW[:3, 3] = (0.01, 0.0175, 0.0125)


@pytest.fixture(scope='module')
def cam():
    return load_script('calibrate_camera')


def rpy(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def floor_depth(pitch_deg, roll_deg, height_m, noise_mm=1.0, seed=0):
    """Render the floor z = 0 (base) as a depth image from a camera at this mount."""
    screw_in_base = np.eye(4)
    screw_in_base[:3, :3] = rpy(math.radians(roll_deg), math.radians(pitch_deg))
    screw_in_base[:3, 3] = (0.25, 0.0, height_m)
    opt_in_base = screw_in_base @ OPT_IN_SCREW
    v, u = np.mgrid[0:480, 0:640]
    rays = np.stack(((u - K[2]) / K[0], (v - K[5]) / K[4], np.ones_like(u, float)), axis=-1)
    r, o = opt_in_base[:3, :3], opt_in_base[:3, 3]
    dirs_z = rays @ r[2]
    with np.errstate(divide='ignore', invalid='ignore'):
        z = np.where(dirs_z < -1e-6, -o[2] / dirs_z, 0.0)
    rng = np.random.default_rng(seed)
    z = z + rng.normal(0, noise_mm / 1000.0, z.shape) * (z > 0)
    depth = np.clip(z * 1000.0, 0, 65535).astype(np.uint16)
    depth[z * 1000.0 > 4000] = 0
    return depth, opt_in_base


@pytest.mark.parametrize(
    'pitch, roll, height', [(0.0, 0.0, 0.10), (8.0, -1.5, 0.12), (-3.0, 2.0, 0.09)]
)
def test_recovers_the_mount(cam, pitch, roll, height):
    depth, opt_in_base = floor_depth(pitch, roll, height)
    pts = cam.deproject(depth, K)
    in_base = cam.transform(pts, opt_in_base)
    ahead = (np.abs(in_base[:, 2]) < 0.1) & (in_base[:, 0] > 0.4) & (in_base[:, 0] < 1.8)
    plane = cam.fit_plane(cam.transform(pts[ahead], OPT_IN_SCREW))
    assert cam.plane_problem(plane) == ''
    mount = cam.mount_from_plane(plane)
    assert mount.pitch_deg == pytest.approx(pitch, abs=0.1)
    assert mount.roll_deg == pytest.approx(roll, abs=0.1)
    assert mount.bottom_z_mm == pytest.approx(height * 1000, abs=1.0)


def test_a_box_in_view_does_not_move_the_floor(cam):
    depth, opt_in_base = floor_depth(5.0, 0.0, 0.10)
    depth[200:300, 250:400] = 700  # a wall-like patch 0.7 m ahead
    pts = cam.deproject(depth, K)
    plane = cam.fit_plane(cam.transform(pts, OPT_IN_SCREW))
    assert cam.mount_from_plane(plane).pitch_deg == pytest.approx(5.0, abs=0.2)


def test_too_little_floor(cam):
    assert cam.plane_problem(None) == 'too little floor in view'


def test_quaternion_matrix(cam):
    from types import SimpleNamespace as N

    half = math.radians(90) / 2
    m = cam.matrix(N(x=1, y=2, z=3), N(x=0, y=0, z=math.sin(half), w=math.cos(half)))
    assert m[:3, :3] @ np.array([1, 0, 0]) == pytest.approx([0, 1, 0])
    assert m[:3, 3].tolist() == [1, 2, 3]


def test_check_tolerances(cam):
    file = {'camera_pitch_deg': 5.0, 'camera_roll_deg': 0.0, 'camera_bottom_z_mm': 100.0}
    good = cam.check_criteria(cam.Mount(5.3, -0.2, 103.0), file)
    assert [c.status for c in good] == ['PASS'] * 3
    bad = cam.check_criteria(cam.Mount(6.0, 0.0, 100.0), file)
    assert bad[0].status == 'FAIL' and 'racecar calibrate camera' in bad[0].advice
