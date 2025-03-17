"""
Copyright (c) 2024, Jose Luis Cercos-Pita <jlc@core-marine.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

def moorpy_export(moorpy_system, filename,
                  version_major=2, version_minor=4):
    """Generate an Initial Condition from a MoorPy already generated and solved
    system

    Parameters
    ----------
    moorpy_system (moorpy.System): The MoorPy instance
    filename (str): The output file path
    version_major (int): MoorDyn major version
    version_minor (int): MoorDyn minor version
    """
    import moorpy
    assert isinstance(moorpy_system, moorpy.System)
    import struct
    from scipy.spatial.transform import Rotation as R
    with open(filename, 'wb') as out:
        # Write the header
        out.write(b'MoorDyn')
        out.write(struct.pack('<B', version_major))
        out.write(struct.pack('<B', version_minor))
        data = b''
        ninstances = 0
        for body in moorpy_system.bodyList:
            ninstances += 1
            r = R.from_matrix(body.R)
            data += struct.pack('<Q', 1)
            data += struct.pack('<Q', 13)
            data += struct.pack('<ddd', *body.r6[:3])
            data += struct.pack('<dddd', *r.as_quat())
            data += struct.pack('<d', 0.0) * 6
        for rod in moorpy_system.rodList:
            ninstances += 1
            if isinstance(rod, moorpy.Line):
                x, y, z, _ = rod.getLineCoords(-1)
                rA = np.asarray([x[0], y[0], z[0]])
                rB = np.asarray([x[-1], y[-1], z[-1]])
                L = rod.L
            else:
                rA = rB = rod.r
                L = 0
            if L > 0.0:
                k = (rB - rA) / L
                Rmat = np.array(rotationMatrix(
                    0,
                    np.arctan2(np.hypot(k[0],k[1]), k[2]),
                    np.arctan2(k[1],k[0])))
                r = R.from_matrix(Rmat)
            else:
                r = R.from_matrix(np.eye(3))
            data += struct.pack('<Q', 1)
            data += struct.pack('<Q', 13)
            data += struct.pack('<ddd', *rA)
            data += struct.pack('<dddd', *r.as_quat())
            data += struct.pack('<d', 0.0) * 6
        for point in moorpy_system.pointList:
            ninstances += 1
            data += struct.pack('<Q', 1)
            data += struct.pack('<Q', 6)
            data += struct.pack('<ddd', *point.r)
            data += struct.pack('<d', 0.0) * 3
        for line in moorpy_system.lineList:
            x, y, z, _ = line.getLineCoords(-1)
            n = len(x)
            ninstances += 1
            data += struct.pack('<Q', n - 2)
            data += struct.pack('<Q', 6)
            for i in range(1, n - 1):
                data += struct.pack('<ddd', x[i], y[i], z[i])
                data += struct.pack('<d', 0.0) * 3
        # Save it
        out.write(struct.pack('<Q', len(data) // 8 + 1))
        out.write(struct.pack('<Q', ninstances))
        out.write(data)


def moorpy_ic(infile,
              outfile=None, tol=0.05, version_major=2, version_minor=4):
    """Generate an Initial Condition from a MoorPy/Moordyn input file. If
    MoorPy fails to solve the equilibrium, no Initial Condition file will be
    printed

    Parameters
    ----------
    infile (str): The output file path

    Keyword arguments
    -----------------
    outfile (str): The output file path. If None, infile with a suffix ".ic"
                   will be considered
    tol (float): The absolute tolerance on positions when calculating
                 equilibrium [m]
    version_major (int): MoorDyn major version
    version_minor (int): MoorDyn minor version

    Returns
    -------
    success (bool): True/False whether converged to within tolerance.
    """
    import moorpy
    print("WARNING: MoorPy does not support all features of MoorDyn, some states may not initialize")
    outfile = outfile or infile + ".ic"
    system = moorpy.System(file=infile)
    system.initialize()
    try:
        success = system.solveEquilibrium(tol=tol)
    except moorpy.SolveError:
        success = False
    if success:
        moorpy_export(system, outfile)
    return success
