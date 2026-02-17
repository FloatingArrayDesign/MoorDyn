/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file c_api.c
 * Check that the C API can be called
 */

#include "MoorDyn.h"
#include "MoorDyn2.h"
#include <stdio.h>

int
main(int narg, char** arg)
{
	int ret_code;
	int n;
	unsigned int un;
	double t=0, dt=0;
	double d;

	// MoorDyn.h
	ret_code = MoorDynInit(NULL, NULL, "nofile");
	if (ret_code != MOORDYN_UNHANDLED_ERROR) {
		printf("MoorDynInit() test failed...");
		return 255;
	}
	ret_code = MoorDynStep(NULL, NULL, NULL, &t, &dt);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDynStep() test failed...");
		return 255;
	}
	ret_code = MoorDynClose();
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDynClose() test failed...");
		return 255;
	}
	ret_code = externalWaveKinInit();
	if (ret_code != MOORDYN_SUCCESS) {
		printf("externalWaveKinInit() test failed...");
		return 255;
	}
	getWaveKinCoordinates(NULL);
	setWaveKin(NULL, NULL, t);
	ret_code = GetFairTen(0);
	if (ret_code != MOORDYN_INVALID_INPUT_FILE) {
		printf("GetFairTen() test failed...");
		return 255;
	}
	ret_code = GetFASTtens(&n, NULL, NULL, NULL, NULL);
	if (ret_code != MOORDYN_MEM_ERROR) {
		printf("GetFASTtens() test failed...");
		return 255;
	}
	ret_code = GetPointForce(0, NULL);
	if (ret_code != MOORDYN_MEM_ERROR) {
		printf("GetPointForce() test failed...");
		return 255;
	}
	ret_code = GetNodePos(0, 0, NULL);
	if (ret_code != MOORDYN_MEM_ERROR) {
		printf("GetNodePos() test failed...");
		return 255;
	}

	// MoorDyn2.h
	MoorDyn system = MoorDyn_Create("nofile");
	if (system) {
		printf("MoorDyn_Create() test failed...");
		return 255;
	}
	ret_code = MoorDyn_NCoupledDOF(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_NCoupledDOF() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SetVerbosity(NULL, n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_SetVerbosity() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SetLogFile(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_SetLogFile() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SetLogLevel(NULL, n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_SetLogLevel() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Log(NULL, n, "nomsg");
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Log() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Init(NULL, NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Init() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Init_NoIC(NULL, NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Init_NoIC() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Step(NULL, NULL, NULL, NULL, &t, &dt);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Step() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Close(NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Close() test failed...");
		return 255;
	}
	MoorDynWaves waves = MoorDyn_GetWaves(NULL);
	if (waves) {
		printf("MoorDyn_GetWaves() test failed...");
		return 255;
	}
	MoorDynSeafloor seafloor = MoorDyn_GetSeafloor(NULL);
	if (seafloor) {
		printf("MoorDyn_GetSeafloor() test failed...");
		return 255;
	}
	ret_code = MoorDyn_ExternalWaveKinInit(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_ExternalWaveKinInit() test failed...");
		return 255;
	}
	ret_code = MoorDyn_ExternalWaveKinGetN(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_ExternalWaveKinGetN() test failed...");
		return 255;
	}
	ret_code = MoorDyn_ExternalWaveKinGetCoordinates(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_ExternalWaveKinGetCoordinates() test failed...");
		return 255;
	}
	ret_code = MoorDyn_ExternalWaveKinSet(NULL, NULL, NULL, t);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_ExternalWaveKinSet() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetNumberBodies(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetNumberBodies() test failed...");
		return 255;
	}
	MoorDynBody body = MoorDyn_GetBody(NULL, un);
	if (body) {
		printf("MoorDyn_GetBody() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetNumberRods(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetNumberRods() test failed...");
		return 255;
	}
	MoorDynRod rod = MoorDyn_GetRod(NULL, un);
	if (rod) {
		printf("MoorDyn_GetRod() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetNumberPoints(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetNumberPoints() test failed...");
		return 255;
	}
	MoorDynPoint point = MoorDyn_GetPoint(NULL, un);
	if (point) {
		printf("MoorDyn_GetPoint() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetNumberLines(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetNumberLines() test failed...");
		return 255;
	}
	MoorDynLine line = MoorDyn_GetLine(NULL, un);
	if (line) {
		printf("MoorDyn_GetLine() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetFASTtens(NULL, &n, NULL, NULL, NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetFASTtens() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Serialize(NULL, NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Serialize() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Deserialize(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Deserialize() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Save(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Save() test failed...");
		return 255;
	}
	ret_code = MoorDyn_Load(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_Load() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SaveVTK(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE && ret_code != MOORDYN_NON_IMPLEMENTED) {
		printf("MoorDyn_SaveVTK() test failed...");
		return 255;
	}

	// Body.h
	ret_code = MoorDyn_GetBodyID(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyID() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyType(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyType() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyState(NULL, NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyState() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyPos(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyPos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyAngle(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyAngle() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyVel(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyVel() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyAngVel(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyAngVel() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyForce(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyForce() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyM(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyM() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyAngVel(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyPos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetBodyAngVel(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetBodyPos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SaveBodyVTK(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE && ret_code != MOORDYN_NON_IMPLEMENTED) {
		printf("MoorDyn_SaveBodyVTK() test failed...");
		return 255;
	}

	// Line.h
	ret_code = MoorDyn_GetLineID(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineID() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineN(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineN() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNumberNodes(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNumberNodes() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineUnstretchedLength(NULL, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineUnstretchedLength() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SetLineUnstretchedLength(NULL, d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_SetLineUnstretchedLength() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SetLineUnstretchedLengthVel(NULL, d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_SetLineUnstretchedLengthVel() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodePos(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodePos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeVel(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodePos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeForce(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeTen(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeBendStiff(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeWeight(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeDrag(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeFroudeKrilov(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeSeabedForce(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeCurv(NULL, un, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeCurv() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineNodeM(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineNodeTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineFairTen(NULL, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineFairTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetLineMaxTen(NULL, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetLineMaxTen() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SaveLineVTK(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE && ret_code != MOORDYN_NON_IMPLEMENTED) {
		printf("MoorDyn_SaveLineVTK() test failed...");
		return 255;
	}

	// Point.h
	ret_code = MoorDyn_GetPointID(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointID() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointType(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointType() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointPos(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointPos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointVel(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointVel() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointForce(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointForce() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointM(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointM() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointNAttached(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointNAttached() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetPointAttached(NULL, un, NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetPointAttached() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SavePointVTK(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE && ret_code != MOORDYN_NON_IMPLEMENTED) {
		printf("MoorDyn_SavePointVTK() test failed...");
		return 255;
	}

	// Rod.h
	ret_code = MoorDyn_GetRodID(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodID() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetRodType(NULL, &n);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodType() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetRodForce(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodForce() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetRodM(NULL, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodM() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetRodN(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodN() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetRodNumberNodes(NULL, &un);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodNumberNodes() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetRodNodePos(NULL, un, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetRodNodePos() test failed...");
		return 255;
	}
	ret_code = MoorDyn_SaveRodVTK(NULL, "nofile");
	if (ret_code != MOORDYN_INVALID_VALUE && ret_code != MOORDYN_NON_IMPLEMENTED) {
		printf("MoorDyn_SaveRodVTK() test failed...");
		return 255;
	}

	// Seafloor.h
	ret_code = MoorDyn_GetDepthAt(NULL, 0.0, 0.0, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetDepthAt() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetAverageDepth(NULL, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetAverageDepth() test failed...");
		return 255;
	}
	ret_code = MoorDyn_GetMinDepth(NULL, &d);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetMinDepth() test failed...");
		return 255;
	}

	// Waves.h
	ret_code = MoorDyn_GetWavesKin(NULL, 0.0, 0.0, 0.0, NULL, NULL, &d, &d, NULL);
	if (ret_code != MOORDYN_INVALID_VALUE) {
		printf("MoorDyn_GetWavesKin() test failed...");
		return 255;
	}
	double k = WaveNumber(0.0, 0.0, 0.0);

	return 0;
}
