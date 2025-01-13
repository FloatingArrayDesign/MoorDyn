#include "MDBench.hpp"
#include <benchmark/benchmark.h>
#include <sstream>

static void
LineGetStateDeriv(benchmark::State& state, std::string input_file)
{

	moordyn::MoorDyn system(input_file.c_str(), MOORDYN_NO_OUTPUT);
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

	auto line = system.GetLines().front();

	std::vector<moordyn::vec> vel{};
	std::vector<moordyn::vec> acc{};
	for (auto _ : state) {
		line->getStateDeriv(vel, acc);
	}
}
BENCHMARK_CAPTURE(LineGetStateDeriv,
                  WaveGrid,
                  "Mooring/wavekin_2/wavekin_2.txt")
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(LineGetStateDeriv, NoWaves, "Mooring/no_waves.txt")
    ->Unit(benchmark::kMicrosecond);

/**
 * @brief Benchmarks stepping the given model 0.1s
 * Loads the given model file and measures the duration
 * to complete an 0.1 second outer time step.
 * @param state
 * @param input_file Path to the model file
 */
static void
MoordynStep(benchmark::State& state, std::string input_file)
{
	moordyn::MoorDyn system(input_file.c_str(), MOORDYN_NO_OUTPUT);
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

	double t = 0.0, dt = 0.1;
	double f[3];

	for (auto _ : state) {
		system.Step(x, dx, f, t, dt);
	}
	state.counters["OuterTimeStep"] = dt;
}

BENCHMARK_CAPTURE(MoordynStep, FloatingRods, "Mooring/floating_rods.txt")
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(MoordynStep, WaveKin0, "Mooring/wavekin_0/wavekin_0.txt")
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(MoordynStep, WaveKin2, "Mooring/wavekin_2/wavekin_2.txt")
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(MoordynStep, WaveKin7, "Mooring/wavekin_7/wavekin_7.txt")
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
