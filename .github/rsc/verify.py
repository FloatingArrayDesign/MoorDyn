import argparse
import os
import sys
import moordyn
import numpy as np

parser = argparse.ArgumentParser(
    description='Run the same MDF regression tests and check')
parser.add_argument('root',
    help='The root folder with both openfast/ and openfast.build/ subfolders')


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
    data = []
    with open(fpath, "r") as fin:
        lines = fin.readlines()[skiplines:]
        for line in lines:
            line = line.strip().replace("\t", " ")
            while line.find("  ") != -1:
                line = line.replace("  ", " ")
            data.append([float(field) for field in line.split()])
    return np.transpose(data)


# Run the tests...
all_good = True
for test in tests:
    print(f"Test {test}...")
    env, md = read_driver(test)
    fname = create_input_file(env, md)
    system = moordyn.Create(fname)
    # Get the NDoFs and check if the motions are right
    ndofs = moordyn.NCoupledDOF(system)
    motions = None
    if md["InputsMode"]:
        motions = interpolate_motions(read_motions(md["InputsFile"]), md)
        assert motions.shape[0] - 1 == ndofs
    # Run the simulation
    r, _ = get_state(system)
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
        moordyn.Step(system, r, u, t, dt)
    moordyn.Close(system)
    # Read the ouputs and compare
    ref = read_outs(md["OutRootName"], skiplines=8)
    new = read_outs(os.path.splitext(fname)[0] + ".out", skiplines=2)
    passing = np.all(pass_fail.passing_channels(ref, new, 2.0, 1.9))
    print("Passed!" if passing else "Failed.")
    if not passing:
        all_good = False

assert all_good
