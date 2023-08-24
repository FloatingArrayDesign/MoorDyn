#include "SpectrumKin.hpp"
#include "Waves.h"
#include "Waves.hpp"
#include "Waves/SpectrumKin.hpp"
#include <complex>

namespace moordyn {

namespace waves {

void
SpectrumKin::setup(const std::vector<FrequencyComponent>& freqComps,
                   EnvCondRef env,
                   SeafloorRef seafloor)
{
	auto num_freqs = freqComps.size();

	omegas = Eigen::ArrayX<real>::Zero(num_freqs);
	amplitudes = Eigen::ArrayX<real>::Zero(num_freqs);
	betas = Eigen::ArrayX<real>::Zero(num_freqs);
	phases = Eigen::ArrayX<real>::Zero(num_freqs);
	kValues = Eigen::ArrayX<real>::Zero(num_freqs);

	for (unsigned int i = 0; i < num_freqs; i++) {
		auto& comp = freqComps[i];
		omegas[i] = comp.omega;
		auto amplitude = abs(comp.amplitude);
		amplitudes[i] = amplitude;
		betas[i] = comp.beta;
		phases[i] = atan2(comp.amplitude.imag(), comp.amplitude.real());

		kValues[i] =
		    WaveNumber(comp.omega,
		               env->g,
		               seafloor ? -seafloor->getAverageDepth() : env->WtrDpth);
	}
}

void
SpectrumKin::getWaveKin(vec3 pos,
                        real t,
                        real avgDepth,
                        real actualDepth,
                        real* zeta,
                        vec3* vel,
                        vec3* acc) const
{
	// const real beta = 0.0;

	const auto& x = pos.x();
	const auto& y = pos.y();

	const Eigen::ArrayX<real> betas_x = betas.cos();
	const Eigen::ArrayX<real> betas_y = betas.sin();
	const Eigen::ArrayX<real> distances = betas_x * x + betas_y * y;
	const Eigen::ArrayX<real> sin_waves =
	    amplitudes * (omegas * t - kValues * distances + phases).sin();
	const Eigen::ArrayX<real> cos_waves =
	    amplitudes * (omegas * t - kValues * distances + phases).cos();

	const real surface_height = sin_waves.sum();
	const real bottom = actualDepth;
	const real actual_depth = surface_height - bottom;

	real stretched_z =
	    (-avgDepth * (pos.z() - bottom)) / actual_depth + avgDepth;

	// If the point is above the water surface, return the values at the water
	// surface. This is important for situations where a point is out of the
	// water but the object itself may have significant submergence (sideway
	// floating line, top node of buoy, etc)
	if (stretched_z > 0.0) {
		stretched_z = 0.0;
	}

	if (zeta) {
		*zeta = surface_height;
	}
	if (vel || acc) {
		vec3 vel_sum = vec3::Zero();
		vec3 acc_sum = vec3::Zero();

		for (unsigned int I = 0; I < omegas.size(); I++) {
			real SINHNumOvrSIHNDen;
			real COSHNumOvrSIHNDen;
			real COSHNumOvrCOSHDen;
			auto k = kValues[I];
			real w = omegas[I];
			if (k == 0.0) {
				// The shallow water formulation is ill-conditioned;
				// thus, the known value of unity is returned.
				SINHNumOvrSIHNDen = 1.0;
				COSHNumOvrSIHNDen = 99999.0;
				COSHNumOvrCOSHDen = 1.0;
			} else if (k * -avgDepth > 89.4) {
				// The shallow water formulation will trigger a
				// floating point overflow error; however, for h
				// > 14.23 * wavelength (since k = 2 * Pi /
				// wavelength) we can use the numerically-stable
				// deep water formulation instead.
				SINHNumOvrSIHNDen = exp(k * stretched_z);
				COSHNumOvrSIHNDen = exp(k * stretched_z);
				COSHNumOvrCOSHDen = exp(k * stretched_z) +
				                    exp(-k * (stretched_z + 2.0 * -avgDepth));
			} else if (-k * -avgDepth > 89.4) {
				// @mth: added negative k case
				// NOTE: CHECK CORRECTNESS
				SINHNumOvrSIHNDen = -exp(-k * stretched_z);
				COSHNumOvrSIHNDen = -exp(-k * stretched_z);
				COSHNumOvrCOSHDen = -exp(-k * stretched_z) +
				                    exp(-k * (stretched_z + 2.0 * -avgDepth));
			} else {
				// shallow water formulation
				SINHNumOvrSIHNDen =
				    sinh(k * (stretched_z + -avgDepth)) / sinh(k * -avgDepth);
				COSHNumOvrSIHNDen =
				    cosh(k * (stretched_z + -avgDepth)) / sinh(k * -avgDepth);
				COSHNumOvrCOSHDen =
				    cosh(k * (stretched_z + -avgDepth)) / cosh(k * -avgDepth);
			}
			real u_xy = w * sin_waves[I] * COSHNumOvrSIHNDen;
			real ux = u_xy * betas_x[I];
			real uy = u_xy * betas_y[I];
			real uz = w * cos_waves[I] * SINHNumOvrSIHNDen;
			vel_sum += vec3(ux, uy, uz);
			real a_xy = w * w * cos_waves[I] * COSHNumOvrSIHNDen;
			real ax = a_xy * betas_x[I];
			real ay = a_xy * betas_y[I];
			real az = -w * w * sin_waves[I] * SINHNumOvrSIHNDen;
			acc_sum += vec3(ax, ay, az);
			// TODO Waves - calculate the dynamic pressure (rods use it)
		}

		if (vel)
			*vel = vel_sum;
		if (acc)
			*acc = acc_sum;
	}
}
}
}
