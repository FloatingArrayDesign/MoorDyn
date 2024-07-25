###############################################################################
# Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
###############################################################################

# This tests are meant to be ran by GitHub actions to compare with the MoorDynF
# implementation, allocated on the OpenFAST repository.
# See .github/workflows/mdf_verification.yml

import argparse
import os
import sys
import moordyn
import numpy as np
try:
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
except ImportError:
    plt = None

parser = argparse.ArgumentParser(
    description='Run the same MDF regression tests and check')
parser.add_argument('root', type=str,
    help='The root folder with both openfast/ and openfast.build/ subfolders')
parser.add_argument('--rtol', type=float, default=1.5,
    help=('Relative tolerance to allow the solution to deviate; expressed as ',
          'order of magnitudes less than baseline'))
parser.add_argument('--atol', type=float, default=1.5,
    help=('Absolute tolerance to allow small values to pass; expressed as ',
          'order of magnitudes less than baseline'))


# Let's start importing some handy tools from MDF
args = parser.parse_args()
sys.path.append(os.path.join(args.root, 'openfast/reg_tests/lib/'))
import pass_fail

# Now get the list of tests to run
mdf_results_root = os.path.join(args.root,
                                'openfast.build/reg_tests/modules/moordyn/')
tests = [f for f in os.listdir(mdf_results_root) if os.path.isdir(
    os.path.join(mdf_results_root, f))]


def mdopts2dict(lines):
    data = {}
    for i, line in enumerate(lines):
        line = line.strip()
        if line == "":
            continue

        def to_num(s):
            try:
                return int(s)
            except ValueError:
                try:
                    return float(s)
                except ValueError:
                    return s

        if line.startswith('"'):
            # The value is a string
            end = line[1:].find('"') + 1 
            value = line[1:end]
        else:
            end = line.find(' ')
            value = to_num(line[:end])

        line = line[end:]
        while line.find("  ") != -1:
            line = line.replace("  ", " ")
        key = line.split(" ")[1]

        data[key] = value

    return data


def read_driver(test):
    # Read the driver test data
    test_root = os.path.join(args.root,
                             'openfast/reg_tests/r-test/modules/moordyn/',
                             test)
    with open(os.path.join(test_root, "md_driver.inp"), "r") as f:
        lines = f.readlines()

    def get_section(lines, name):
        start, end = None, None
        for i, line in enumerate(lines):
            line = line.strip()
            if not line.startswith("---"):
                continue
            if start is None and name in line:
                start = i + 1
                continue
            if start is not None and end is None:
                end = i
                break
        assert start is not None
        end = end or len(lines)
        assert start < end
        return lines[start:end]

    env = mdopts2dict(get_section(lines, "ENVIRONMENTAL CONDITIONS"))
    md = mdopts2dict(get_section(lines, "MOORDYN"))
    for filepaths in ("MDInputFile", "InputsFile", "OutRootName"):
        if md[filepaths]:
            md[filepaths] = os.path.join(test_root, md[filepaths])
    md["OutRootName"] = md["OutRootName"] + ".MD.out"
    return env, md


def create_input_file(env, md):
    fname = os.path.basename(md["MDInputFile"])
    with open(md["MDInputFile"], "r") as fin, open(fname, "w") as fout:
        lines = fin.readlines()
        # Find the options section and append the env options (if required)
        for i, line in enumerate(lines):
            line = line.strip()
            if line.startswith('---') and "OPTIONS" in line:
                start = i + 1
                break
        opts = lines[start:]
        end = len(opts)
        for i, line in enumerate(opts):
            line = line.strip()
            if line.startswith('---'):
                end = i
                break
        opts = mdopts2dict(opts[:end])
        for optin, optout in (('Gravity', 'gravity'),
                              ('rhoW', 'rho'),
                              ('WtrDpth', 'WtrDpth')):
            if optin not in env.keys():
                if optout not in opts.keys():
                    print(f"WARNING: Environmental option {optin} not defined")
                continue
            if optout in opts.keys():
                # Warn about the conflict. We will anyway overwrite the option
                if env[optin] != opts[optout]:
                    print(
                        f"WARNING: {optin}={env[optin]} in MDF driver vs. " + \
                        f"{optout}={opts[optout]} in MoorDyn config file")
            lines.insert(start + end,
                         f"{env[optin]}  {optout}  option set by the driver\n")
        for line in lines:
            fout.write(line)
    return fname
        

def read_motions(fpath):
    data = []
    with open(fpath, "r") as fin:
        lines = fin.readlines()
        for line in lines:
            line = line.strip()
            if line.startswith("#") or line == "":
                continue
            while line.find("  ") != -1:
                line = line.replace("  ", " ")
            fields = [float(field) for field in line.split()]
            if len(data):
                assert len(data[-1]) == len(fields)
            data.append(fields)
    return np.transpose(data)


def interpolate_motions(motions, md):
    t = np.arange(0, md["TMax"] + md["dtC"], md["dtC"])
    new_motions = [t]
    for i in range(1, motions.shape[0]):
        new_motions.append(np.interp(t, motions[0, :], motions[i, :]))
    return np.asarray(new_motions)


def get_state(system):
    """ Get all the state variables, as they would be used on moordyn.Init()
    and moordyn.Step()

    Parameters
    ==========

    system (cmoordyn.MoorDyn): The MoorDyn instance

    Returns
    =======

    state (lst): The list with the sate variables
    names (lst): The list of names that each state variable belongs to
    """
    state = []
    names = []
    n_bodies = moordyn.GetNumberBodies(system)
    for i in range(n_bodies):
        body = moordyn.GetBody(system, i + 1)
        if moordyn.GetBodyType(body) not in (moordyn.BODY_TYPE_COUPLED,):
            continue
        body_id = moordyn.GetBodyID(body)
        body_state = moordyn.GetBodyState(body)
        
        state += np.array(body_state).flatten().tolist()
        names += [f"body_{body_id}",] * len(body_state)
    n_rods = moordyn.GetNumberRods(system)
    for i in range(n_rods):
        rod = moordyn.GetRod(system, i + 1)
        if moordyn.GetRodType(rod) not in (moordyn.ROD_TYPE_COUPLED,
                                           moordyn.ROD_TYPE_CPLDPIN):
            continue
        rod_id = moordyn.GetRodID(rod)
        rod_state = moordyn.GetRodState(rod)
        state += np.array(rod_state).flatten().tolist()
        names += [f"rod_{rod_id}",] * len(rod_state)
    n_points = moordyn.GetNumberPoints(system)
    for i in range(n_points):
        point = moordyn.GetPoint(system, i + 1)
        if moordyn.GetPointType(point) not in (moordyn.POINT_TYPE_COUPLED, ):
            continue
        point_id = moordyn.GetPointID(point)
        point_state = moordyn.GetPointPos(point)
        state += point_state
        names += [f"point_{point_id}",] * len(point_state)
    return state, names


def read_outs(fpath, skiplines=2):
    def to_num(s):
        if '***' in s:
            return 0.0
        return float(s)

    data = []
    with open(fpath, "r") as fin:
        lines = fin.readlines()[skiplines:]
        for line in lines:
            line = line.strip().replace("\t", " ")
            while line.find("  ") != -1:
                line = line.replace("  ", " ")
            data.append([to_num(field) for field in line.split()])
    return np.transpose(data)


def plot(ref, data, fpath):
    if plt is None:
        return
    colors = list(mcolors.XKCD_COLORS.values())
    for i in range(1, ref.shape[0]):
        plt.plot(ref[0, :], ref[i, :], linestyle='dashed',
                 color=colors[i - 1])
        plt.plot(data[0, :], data[i, :], linestyle='solid',
                 color=colors[i - 1], label=f'channel {i}')
    plt.legend(loc='best')
    plt.savefig(fpath)


# Run the tests...
summary = {}
for test in tests:
    print(f"Test {test}...")
    env, md = read_driver(test)
    fname = create_input_file(env, md)
    system = moordyn.Create(fname)
    # Get the NDoFs and check if the motions are right
    ndofs = moordyn.NCoupledDOF(system)
    motions = None
    if md["InputsMode"] and md["InputsFile"]:
        motions = interpolate_motions(read_motions(md["InputsFile"]), md)
        assert motions.shape[0] - 1 == ndofs
    # Run the simulation
    r = get_state(system)[0] if motions is None else motions[1:, 0]
    u = [0 for i in range(ndofs)]
    moordyn.Init(system, r, u)
    dt = md["dtC"]
    T = md["TMax"]
    tlist = np.arange(0, T, dt)
    for i, t in enumerate(tlist):
        rorg, _ = get_state(system)
        rorg = np.asarray(rorg)
        rdst = rorg if motions is None else motions[1:, i + 1]
        u = (rdst - rorg) / dt
        moordyn.Step(system, rorg, u, t, dt)
    moordyn.Close(system)
    # Read the ouputs and compare
    ref = read_outs(md["OutRootName"], skiplines=8)
    new = read_outs(os.path.splitext(fname)[0] + ".out", skiplines=2)
    # Drop the eventual points at the tail that ight come from precision errors
    # on the time
    n_samples = min(ref.shape[1], new.shape[1])
    ref = ref[:, :n_samples]
    new = new[:, :n_samples]
    plot(ref, new, test + ".png")
    passing = np.all(
        pass_fail.passing_channels(ref, new, args.rtol, args.atol))
    
    summary[test] = passing

print("")
print("=" * 80)
values = list(summary.values())
N = len(values)
if not np.all(values):
    n = N - np.sum(list(values))
    print(f"{n} / {N} tests failed:")
    for i, (key, value) in enumerate(summary.items()):
        print(f"\t{i + 1}: Test '{key}' " + ("PASSED" if value else "FAILED"))
else:
    print(f"{N} / {N} tests passed:")
print("=" * 80)
print("")

assert np.all(values)
