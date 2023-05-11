#include <benchmark/benchmark.h>
#include "WaveBench.hpp"
#include "Waves/SpectrumKin.hpp"
#include <random>

moordyn::waves::SpectrumKin
generate_spectrum_kin()
{

	auto log = moordyn::Log(MOORDYN_NO_OUTPUT, MOORDYN_NO_OUTPUT);
	moordyn::Waves waves(&log);
	auto spectrum = moordyn::waves::spectrumFromFile(
	    "Mooring/ten_wave_frequencies.txt", &log);

	EnvCondRef env = make_shared<EnvCond>();
	env->g = 9.8;
	env->WtrDpth = 50;
	moordyn::waves::SpectrumKin spectrumKin;
	spectrumKin.setup(spectrum.getComponents(), env, nullptr);

	return spectrumKin;
}

/**
 * @brief Benchmarks calculating the water kinematics from a 10 component wave
 * spectrum
 *
 * Requests that the surface height be returned (this means that the velocity
 * and acceleration won't be calculated)
 *
 * @param state
 */
static void
BM_SpectrumKinZeta(benchmark::State& state)
{
	auto spectrumKin = generate_spectrum_kin();

	moordyn::real x = 0.0;
	moordyn::real y = 0.0;
	moordyn::real z = -10.0;
	moordyn::real t = 0.0;
	moordyn::real zeta;
	for (auto _ : state) {
		spectrumKin.getWaveKin(
		    moordyn::vec3(x, y, z), t, -50, -50, &zeta, nullptr, nullptr);
	}
}

BENCHMARK(BM_SpectrumKinZeta);

/**
 * @brief Benchmarks calculating the water kinematics from a 10 component wave
 * spectrum
 *
 * Requests that the velocity be returned (preventing any early exiting)
 *
 * @param state
 */
static void
BM_SpectrumKinVel(benchmark::State& state)
{
	auto spectrumKin = generate_spectrum_kin();

	moordyn::real x = 0.0;
	moordyn::real y = 0.0;
	moordyn::real z = -10.0;
	moordyn::real t = 0.0;
	moordyn::vec3 vel;
	for (auto _ : state) {
		spectrumKin.getWaveKin(
		    moordyn::vec3(x, y, z), t, -50, -50, nullptr, &vel, nullptr);
	}
}

BENCHMARK(BM_SpectrumKinVel);

/**
 * @brief Benchmarks calculating sinh(k * (z + h)) / sinh(k * h)
 *
 * This calculation is a significant portion of the spectrumKin calculation time
 * so it's a useful frame of reference
 * @param state
 */
static void
BenchScalingFactor(benchmark::State& state)
{
	double k = 1.5;
	double h = 48.0;
	double z = -5.0;
	// this prevents the compiler from compile time optimizing any of these
	// variables away
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-1.0, 1.0);
	k += distribution(generator);
	h += distribution(generator);
	z += distribution(generator);

	double scale;
	for (auto _ : state) {
		benchmark::DoNotOptimize(scale = sinh(k * (z + h)) / sinh(k * h));
		benchmark::ClobberMemory();
	}
}

BENCHMARK(BenchScalingFactor);

BENCHMARK_MAIN();
