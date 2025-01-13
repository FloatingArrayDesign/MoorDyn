#include "LinesBench.hpp"
#include "Time.hpp"
#include <benchmark/benchmark.h>
#include <sstream>

/**
 * @brief Benchmarks stepping the model 0.1s
 * Loads the given model file and measures the duration
 * to complete an 0.1 second outer time step.
 * @param state Benchmark state
 * @param tScheme name of integrator to use
 */
static void
MoorDynStepWithIntegrator(benchmark::State& state, const std::string& tScheme)
{
	int num_lines = state.range(0);
	int num_segments = state.range(1);
	std::string input_file = "Mooring/cases/" + std::to_string(num_lines) +
	                         "_lines_" + std::to_string(num_segments) +
	                         "_segs/lines.txt";
	moordyn::MoorDyn system(input_file.c_str(), MOORDYN_NO_OUTPUT);
	system.SetDisableOutput(true);

	// Don't have to free this pointer because it's done by the MoorDyn system.
	moordyn::TimeScheme* timeScheme = moordyn::create_time_scheme(
	    tScheme, system.GetLogger(), system.GetWaves());
	system.SetTimeScheme(timeScheme);

	// int err;
	unsigned int n_dof;
	n_dof = system.NCoupledDOF();
	double *x = NULL, *dx = NULL;
	if (n_dof) {
		x = new double[n_dof];
		std::fill(x, x + n_dof, 0.0);
		dx = new double[n_dof];
		std::fill(dx, dx + n_dof, 0.0);
	}
	system.Init(x, dx, true);

	double t = 0.0, dt = 0.01;
	double f[3];

	for (auto _ : state) {
		system.Step(x, dx, f, t, dt);
	}
	state.counters["OuterTimeStep"] = dt;
}
static void
MoordynStepCaseRK4(benchmark::State& state)
{
	MoorDynStepWithIntegrator(state, "RK4");
}
BENCHMARK(MoordynStepCaseRK4)
    ->ArgsProduct({ { 1, 2, 4, 8, 16, 32, 64 }, { 1, 2, 4, 8, 16, 32, 64 } })
    ->Unit(benchmark::kMicrosecond);

static void
MoordynStepCaseRK2(benchmark::State& state)
{
	MoorDynStepWithIntegrator(state, "RK2");
}
BENCHMARK(MoordynStepCaseRK2)
    ->ArgsProduct({ { 1, 2, 4, 8, 16, 32, 64 }, { 1, 2, 4, 8, 16, 32, 64 } })
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
