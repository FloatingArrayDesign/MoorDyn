import sys
from unittest import TestCase, main as unittest_main
import os
import moordyn
from scipy.spatial.transform import Rotation as R
import numpy as np
from test_minimal import setup_case


ENUREF = [[1, 0, 0],
          [0, 1, 0],
          [0, 0, 1]]
ANGLE = 5


def rotate_vecs(vecs, angles, seq="XYZ", degrees=True):
    r = R.from_euler(seq, angles, degrees=degrees)
    return [r.apply(vec) for vec in vecs]


def compare_vecs(vecs, refs, tol=1e-5):
    assert len(vecs) == len(refs)
    for i in range(len(vecs)):
        assert len(vecs[i]) == len(refs[i])
        for j in range(len(vecs[i])):
            assert abs(vecs[i][j] - refs[i][j]) < 1e-5


def abseuler2releuler(org, dst, seq="XYZ", degrees=True):
    r0 = R.from_euler(seq, org, degrees=degrees).inv()
    r1 = R.from_euler(seq, dst, degrees=degrees)
    return (r1 * r0).as_euler(seq, degrees=degrees)
    

def rotate_moordyn(system, roll, pitch, yaw, t, dt=1.0, degrees=True):
    if degrees:
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)
    pts = [moordyn.GetPoint(system, i + 1) for i in range(3)]
    body = moordyn.GetBody(system, 1)
    r, _ = moordyn.GetBodyState(body)
    u = [0] * 3 + [roll / dt, pitch / dt, yaw / dt]
    moordyn.Step(system, r, u, t, dt)
    t += dt
    r = [moordyn.GetPointPos(pt) for pt in pts]
    return r, t


class RotationTests(TestCase):
    def setUp(self):
        pass

    def test_nwu(self):
        tmp_folder = setup_case("rotations/nwu.txt")
        cwd = os.getcwd()
        os.chdir(tmp_folder)
        system = moordyn.Create()
        os.chdir(cwd)
        pts = [moordyn.GetPoint(system, i + 1) for i in range(3)]
        r = [0] * 6
        u = [0] * 6
        moordyn.Init(system, r, u)
        # Check the intial condition
        compare_vecs([moordyn.GetPointPos(pt) for pt in pts], ENUREF)
        t = 0
        # Roll test
        r, t = rotate_moordyn(system, ANGLE, 0, 0, t)
        compare_vecs(r, rotate_vecs(ENUREF, ANGLE, seq='X'))
        assert(r[1][2] > 0.0)  # Portside up, i.e. starboard down
        r, t = rotate_moordyn(system, -ANGLE, 0, 0, t)
        compare_vecs(r, ENUREF)
        # Pitch test
        r, t = rotate_moordyn(system, 0, ANGLE, 0, t)
        compare_vecs(r, rotate_vecs(ENUREF, ANGLE, seq='Y'))
        assert(r[0][2] < 0.0)  # Bow down
        r, t = rotate_moordyn(system, 0, -ANGLE, 0, t)
        compare_vecs(r, ENUREF)
        # Yaw test
        r, t = rotate_moordyn(system, 0, 0, ANGLE, t)
        compare_vecs(r, rotate_vecs(ENUREF, ANGLE, seq='Z'))
        assert(r[0][1] > 0.0)  # Bow to portside
        r, t = rotate_moordyn(system, 0, 0, -ANGLE, t)
        compare_vecs(r, ENUREF)

        # Test the angles order. For that we are rotating it in all directions
        # simultaneously
        r, t = rotate_moordyn(system, ANGLE, ANGLE, ANGLE, t)
        compare_vecs(r, rotate_vecs(ENUREF, [ANGLE, ANGLE, ANGLE]))
        r, t = rotate_moordyn(system, -ANGLE, -ANGLE, -ANGLE, t)
        compare_vecs(r, ENUREF)

        # Test the rotations concatenations. To do that we are again rotating on
        # the 3 axis simultaneously. However, this time we rotate twice, so the
        # final angle shall be 2*ANGLE in all directions
        rotate_moordyn(system, ANGLE, ANGLE, ANGLE, t)
        r, t = rotate_moordyn(system, ANGLE, ANGLE, ANGLE, t)
        roll, pitch, yaw = abseuler2releuler([ANGLE, ANGLE, ANGLE],
                                            [2 * ANGLE, 2 * ANGLE, 2 * ANGLE])
        compare_vecs(r, rotate_vecs(rotate_vecs(ENUREF, [ANGLE, ANGLE, ANGLE]),
                                    [roll, pitch, yaw]))
        compare_vecs(r, rotate_vecs(ENUREF, [2 * ANGLE, 2 * ANGLE, 2 * ANGLE]))

        moordyn.Close(system)


if __name__ == '__main__':
    unittest_main()
