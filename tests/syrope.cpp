#include "MoorDyn2.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

// -----------------------------
// Constants
// -----------------------------
constexpr double kG = 9.80665;
constexpr double kMBL = 885.0e3 * kG;

// initial conditions
constexpr double kTmax0 = 0.15 * kMBL;
constexpr double kTmean0 = 0.05 * kMBL; // currently unused

// dynamic coefficients (currently unused)
constexpr double kAlpha = 17.667 * kMBL;
constexpr double kBeta = 0.2313 * 100.0;

enum class WorkingCurveForm
{
	Linear,
	Quadratic,
	Exponential
};

// working curve coefficients
constexpr double kLinearK1 = 1.25e8;
constexpr double kLinearK2 = 0.0;
constexpr double kQuadraticK1 = 0.25;
constexpr double kQuadraticK2 = 0.80;
constexpr double kExpK1 = 0.20;
constexpr double kExpK2 = 1.10;

// Tolerance for regression (adjust as appropriate once you have baselines)
constexpr double kL2Tol = 0.05;

// -----------------------------
// working curve cases
// -----------------------------
struct WcCase
{
	std::string name;
	std::string input_file;
	WorkingCurveForm form;
	double eps_0;
};

// -----------------------------
// MoorDyn error handling
// -----------------------------
inline void
check_md(const int err, const std::string& msg)
{
	REQUIRE(err == MOORDYN_SUCCESS);
	if (err != MOORDYN_SUCCESS) {
		throw std::runtime_error(msg);
	}
}

struct MoorDynRAII
{
	explicit MoorDynRAII(const std::string& input_file)
	  : sys(MoorDyn_Create(input_file.c_str()))
	{
		REQUIRE(sys != nullptr);
		if (!sys) {
			throw std::runtime_error("MoorDyn_Create failed for input: " +
			                         input_file);
		}
	}

	~MoorDynRAII()
	{
		if (sys) {
			(void)MoorDyn_Close(sys);
		}
	}

	MoorDynRAII(const MoorDynRAII&) = delete;
	MoorDynRAII& operator=(const MoorDynRAII&) = delete;

	MoorDyn sys = nullptr;
};

// -----------------------------
// Linear interpolation (clamped)
// xdata must be monotone increasing
// -----------------------------
double
interpolate_clamped(double x,
                    const Eigen::VectorXd& xdata,
                    const Eigen::VectorXd& ydata)
{
	if (xdata.size() != ydata.size()) {
		throw std::invalid_argument("interpolate_clamped: size mismatch");
	}
	if (xdata.size() < 2) {
		throw std::invalid_argument("interpolate_clamped: need >= 2 points");
	}

	for (Eigen::Index i = 1; i < xdata.size(); ++i) {
		if (xdata[i] < xdata[i - 1]) {
			throw std::invalid_argument(
			    "interpolate_clamped: xdata must be monotone increasing");
		}
	}

	if (x <= xdata[0])
		return ydata[0];
	if (x >= xdata[xdata.size() - 1])
		return ydata[ydata.size() - 1];

	Eigen::Index lo = 0;
	Eigen::Index hi = xdata.size() - 1;

	while (hi - lo > 1) {
		const Eigen::Index mid = lo + (hi - lo) / 2;
		if (xdata[mid] <= x)
			lo = mid;
		else
			hi = mid;
	}

	const double x0 = xdata[lo], x1 = xdata[hi];
	const double y0 = ydata[lo], y1 = ydata[hi];
	const double dx = x1 - x0;

	if (std::abs(dx) <= std::numeric_limits<double>::epsilon()) {
		return 0.5 * (y0 + y1);
	}

	const double t = (x - x0) / dx;
	return y0 + t * (y1 - y0);
}

// -----------------------------
// Mean tension from working curve
// -----------------------------
double
find_mean_tension(double strain,
                  double Tmean,
                  double Tmax,
                  const Eigen::VectorXd& owc_strains,
                  const Eigen::VectorXd& owc_tensions,
                  WorkingCurveForm wc_form)
{
	double k1 = 0.0;
	double k2 = 0.0;
	switch (wc_form) {
		case WorkingCurveForm::Linear:
			k1 = kLinearK1;
			k2 = kLinearK2;
			break;
		case WorkingCurveForm::Quadratic:
			k1 = kQuadraticK1;
			k2 = kQuadraticK2;
			break;
		case WorkingCurveForm::Exponential:
			k1 = kExpK1;
			k2 = kExpK2;
			break;
		default:
			throw std::invalid_argument(
			    "find_mean_tension: unknown WorkingCurveForm");
	}

	// If not unloading/reloading, mean tension follows the original working
	// curve
	if (Tmean >= Tmax) {
		return interpolate_clamped(strain, owc_strains, owc_tensions);
	}

	// Unloading/reloading: build working curve between eps_min and eps_max
	const double eps_max = interpolate_clamped(Tmax, owc_tensions, owc_strains);
	const double eps0 = owc_strains[0];

	double eps_min = eps0 + k1 * (eps_max - eps0);

	// Linear WC special rule: if k1 >= 1.0 treat as slope (dimensional
	// stiffness)
	if (wc_form == WorkingCurveForm::Linear && k1 >= 1.0) {
		eps_min = eps_max - Tmax / k1;
	}

	const double denom = eps_max - eps_min;
	if (std::abs(denom) <= std::numeric_limits<double>::epsilon()) {
		return interpolate_clamped(strain, owc_strains, owc_tensions);
	}

	double xi = (strain - eps_min) / denom;
	xi = std::clamp(xi, 0.0, 1.0);

	switch (wc_form) {
		case WorkingCurveForm::Linear:
			return Tmax * xi;
		case WorkingCurveForm::Quadratic:
			return Tmax * xi * (k2 * xi + (1.0 - k2));
		case WorkingCurveForm::Exponential: {
			const double den = std::exp(k2) - 1.0;
			if (std::abs(den) < 1e-14)
				return Tmax * xi; // limit -> linear
			return Tmax * (std::exp(k2 * xi) - 1.0) / den;
		}
		default:
			throw std::invalid_argument(
			    "find_mean_tension: unknown WorkingCurveForm");
	}
}

// -----------------------------
// Read OWC table: 2 header lines then two numeric columns
// -----------------------------
std::pair<Eigen::VectorXd, Eigen::VectorXd>
read_strain_tension_table(const std::string& path)
{
	std::ifstream in(path);
	if (!in) {
		throw std::runtime_error("Cannot open file: " + path);
	}

	std::string line;
	if (!std::getline(in, line))
		throw std::runtime_error("Missing header line 1 in: " + path);
	if (!std::getline(in, line))
		throw std::runtime_error("Missing header line 2 in: " + path);

	std::vector<double> strain, tension;
	strain.reserve(1024);
	tension.reserve(1024);

	std::size_t lineno = 2;
	while (std::getline(in, line)) {
		++lineno;
		if (line.empty())
			continue;

		const auto first = line.find_first_not_of(" \t\r");
		if (first == std::string::npos)
			continue;
		const char c0 = line[first];
		if (c0 == '#' || c0 == '!')
			continue;

		std::istringstream iss(line);
		double eps = 0.0, ten = 0.0;
		if (!(iss >> eps >> ten)) {
			throw std::runtime_error("Failed to parse line " +
			                         std::to_string(lineno) + " in " + path +
			                         ": '" + line + "'");
		}
		strain.push_back(eps);
		tension.push_back(ten);
	}

	if (strain.empty()) {
		throw std::runtime_error("No data rows found in: " + path);
	}

	Eigen::VectorXd eps(static_cast<Eigen::Index>(strain.size()));
	Eigen::VectorXd ten(static_cast<Eigen::Index>(tension.size()));
	for (Eigen::Index i = 0; i < eps.size(); ++i) {
		eps[i] = strain[static_cast<std::size_t>(i)];
		ten[i] = tension[static_cast<std::size_t>(i)];
	}

	return { eps, ten };
}

// -----------------------------
// Read Seg<digits>Te column from MoorDyn output file
// Assumes:
//   1) header tokens line
//   2) units tokens line, with "(N)" under SegNTe
// -----------------------------
Eigen::VectorXd
load_seg_te_column(const std::string& path)
{
	std::ifstream in(path);
	if (!in) {
		throw std::runtime_error("Cannot open " + path);
	}

	std::string line;

	// Header line
	if (!std::getline(in, line)) {
		throw std::runtime_error("Missing header line in " + path);
	}

	std::istringstream hdr(line);
	std::vector<std::string> headers;
	for (std::string tok; hdr >> tok;)
		headers.push_back(tok);
	if (headers.empty()) {
		throw std::runtime_error("Empty header line in " + path);
	}

	// Find first Seg<digits>Te
	const std::regex re(R"(^Seg\d+Te$)");
	int col = -1;
	for (size_t i = 0; i < headers.size(); ++i) {
		if (std::regex_match(headers[i], re)) {
			col = static_cast<int>(i);
			break;
		}
	}
	if (col < 0) {
		throw std::runtime_error("No Seg<digits>Te column found in " + path);
	}

	// Units line
	if (!std::getline(in, line)) {
		throw std::runtime_error("Missing units line in " + path);
	}

	std::istringstream units_iss(line);
	std::vector<std::string> units;
	for (std::string tok; units_iss >> tok;)
		units.push_back(tok);

	if (static_cast<int>(units.size()) <= col) {
		throw std::runtime_error(
		    "Units line has fewer columns than header in " + path);
	}
	if (units[static_cast<size_t>(col)] != "(N)") {
		throw std::runtime_error("Unexpected unit for " +
		                         headers[static_cast<size_t>(col)] + ": got '" +
		                         units[static_cast<size_t>(col)] +
		                         "', expected '(N)' in " + path);
	}

	// Data
	std::vector<double> vals;
	vals.reserve(2048);

	std::size_t lineno = 2;
	while (std::getline(in, line)) {
		++lineno;
		if (line.empty())
			continue;

		std::istringstream row(line);
		double v = 0.0;
		for (int i = 0; i <= col; ++i) {
			if (!(row >> v)) {
				throw std::runtime_error(
				    "Row has fewer numeric columns than expected at line " +
				    std::to_string(lineno) + " in " + path + ": '" + line +
				    "'");
			}
		}
		vals.push_back(v);
	}

	Eigen::VectorXd out(static_cast<Eigen::Index>(vals.size()));
	for (Eigen::Index i = 0; i < out.size(); ++i) {
		out[i] = vals[static_cast<std::size_t>(i)];
	}
	return out;
}

// -----------------------------
// JONSWAP PSD (as in your original code)
// -----------------------------
double
jonswap_psd(double Hs,
            double Tp,
            double gamma,
            double omega,
            double sigma_a = 0.07,
            double sigma_b = 0.09)
{
	if (!(omega > 0.0) || !(Tp > 0.0) || !(gamma > 0.0) || !(Hs >= 0.0)) {
		return 0.0;
	}

	const double wp = 2.0 * M_PI / Tp;
	const double A = 1.0 - 0.287 * std::log(gamma);
	const double sigma = (omega > wp) ? sigma_b : sigma_a;
	const double rw = (omega - wp) / (sigma * wp);
	const double a = std::pow(wp / omega, 4.0);

	return A * (5.0 / 16.0) * Hs * Hs * (a / omega) * std::exp(-1.25 * a) *
	       std::pow(gamma, std::exp(-0.5 * rw * rw));
}

// -----------------------------
// Slow strain profile
// -----------------------------
double
slow_strain(double t, double seg_dur, double eps0)
{
	if (seg_dur <= 0.0)
		return eps0;

	if (t < 1.0 * seg_dur)
		return eps0;
	if (t < 2.0 * seg_dur)
		return eps0 + 2.0 * eps0 * (t - 1.0 * seg_dur) / seg_dur;
	if (t < 3.0 * seg_dur)
		return eps0 + 2.0 * eps0;
	if (t < 4.0 * seg_dur)
		return eps0 + 2.0 * eps0 - 1.0 * eps0 * (t - 3.0 * seg_dur) / seg_dur;
	if (t < 5.0 * seg_dur)
		return eps0 + 1.0 * eps0;
	return eps0 + 1.0 * eps0 + 1.5 * eps0 * (t - 5.0 * seg_dur) / seg_dur;
}

// -----------------------------
// Surface elevation from JONSWAP spectrum
// Deterministic random phases via RNG seed
// -----------------------------
Eigen::VectorXd
surface_elevation_from_jonswap(double Hs,
                               double Tp,
                               double gamma,
                               const Eigen::VectorXd& times,
                               unsigned int n_omega_edges,
                               double omega_min,
                               double omega_max,
                               std::mt19937& rng)
{
	if (n_omega_edges < 2) {
		throw std::runtime_error("n_omega_edges must be >= 2");
	}
	if (!(omega_min > 0.0 && omega_max > omega_min)) {
		throw std::runtime_error("Invalid omega_min/omega_max");
	}

	const Eigen::VectorXd omega_edges =
	    Eigen::VectorXd::LinSpaced(n_omega_edges, omega_min, omega_max);
	const double domega = omega_edges[1] - omega_edges[0];

	const Eigen::VectorXd omega_mid =
	    0.5 * (omega_edges.head(n_omega_edges - 1) +
	           omega_edges.tail(n_omega_edges - 1));

	const Eigen::VectorXd S = omega_mid.unaryExpr(
	    [&](double om) { return jonswap_psd(Hs, Tp, gamma, om); });

	const Eigen::VectorXd a = (2.0 * S * domega).cwiseSqrt();

	std::uniform_real_distribution<double> unif(0.0, 2.0 * M_PI);
	Eigen::VectorXd phi(n_omega_edges - 1);
	for (Eigen::Index i = 0; i < phi.size(); ++i) {
		phi[i] = unif(rng);
	}

	Eigen::VectorXd eta = Eigen::VectorXd::Zero(times.size());
	for (Eigen::Index i = 0; i < times.size(); ++i) {
		const double t = times[i];
		eta[i] = ((omega_mid * t + phi).array().cos() * a.array()).sum();
	}
	return eta;
}

// Helper: "<input_file_name_without_extension>_Line1.txt" without <filesystem>
std::string
line1_output_from_input(const std::string& input_path)
{
	const std::size_t sep = input_path.find_last_of("/\\");
	const std::size_t base = (sep == std::string::npos) ? 0 : (sep + 1);

	const std::size_t dot = input_path.find_last_of('.');
	const bool has_ext = (dot != std::string::npos) && (dot > base);

	const std::string stem = has_ext ? input_path.substr(0, dot) : input_path;
	return stem + "_Line1.out";
}

// -----------------------------
// Run a single case and return relative L2 error
// -----------------------------
double
run_case(const WcCase& c,
         const Eigen::VectorXd& owc_strains,
         const Eigen::VectorXd& owc_tensions)
{
	MoorDynRAII md(c.input_file);

	unsigned int ndof = 0;
	check_md(MoorDyn_NCoupledDOF(md.sys, &ndof),
	         "MoorDyn_NCoupledDOF failed for: " + c.input_file);
	INFO("ndof=" << ndof);

	auto fairlead = MoorDyn_GetPoint(md.sys, 2);
	auto line = MoorDyn_GetLine(md.sys, 1);
	REQUIRE(fairlead != nullptr);
	REQUIRE(line != nullptr);

	double r[3] = { 0.0, 0.0, 0.0 };
	double dr[3] = { 0.0, 0.0, 0.0 };
	check_md(MoorDyn_GetPointPos(fairlead, r),
	         "MoorDyn_GetPointPos failed for: " + c.input_file);

	// time settings
	const double seg_dur = 3.0 * 3600.0;
	const double total_dur = 6.0 * seg_dur;
	const double dt0 = 0.1;
	const unsigned int nsteps = static_cast<unsigned int>(total_dur / dt0);

	const Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(
	    static_cast<Eigen::Index>(nsteps) + 1, 0.0, total_dur);

	// deterministic excitation
	constexpr unsigned int n_omega_edges = 300;
	std::mt19937 rng_wf(12345);
	std::mt19937 rng_lf(54321);

	// WF
	const double Hs_WF = 5.0;
	const double Tp_WF = 12.0;
	const double Tz_WF = Tp_WF / 1.402;
	const double omega_min_WF = 1.0 / Tz_WF;
	const double omega_max_WF = 20.0 / Tz_WF;

	const Eigen::VectorXd eta_WF = surface_elevation_from_jonswap(Hs_WF,
	                                                              Tp_WF,
	                                                              3.3,
	                                                              times,
	                                                              n_omega_edges,
	                                                              omega_min_WF,
	                                                              omega_max_WF,
	                                                              rng_wf);

	// LF
	const double Hs_LF = 20.0;
	const double Tp_LF = 120.0;
	const double Tz_LF = Tp_LF / 1.402;
	const double omega_min_LF = 1.0 / Tz_LF;
	const double omega_max_LF = 20.0 / Tz_LF;

	const Eigen::VectorXd eta_LF = surface_elevation_from_jonswap(Hs_LF,
	                                                              Tp_LF,
	                                                              3.3,
	                                                              times,
	                                                              n_omega_edges,
	                                                              omega_min_LF,
	                                                              omega_max_LF,
	                                                              rng_lf);

	// prescribed strain and x(t)
	const double x0 = 1.0;
	const double scale_WF = 4.00e-4;
	const double scale_LF = 0.80e-4;

	Eigen::VectorXd strain_slow(times.size());
	for (Eigen::Index i = 0; i < times.size(); ++i) {
		strain_slow[i] = slow_strain(times[i], seg_dur, c.eps_0);
	}

	const Eigen::VectorXd strain =
	    strain_slow + scale_WF * eta_WF + scale_LF * eta_LF;
	const Eigen::VectorXd x =
	    x0 * (Eigen::VectorXd::Ones(strain.size()) + strain);

	// init moordyn
	r[0] = x[0];
	dr[0] = (x[1] - x[0]) / dt0;
	dr[1] = 0.0;
	dr[2] = 0.0;

	check_md(MoorDyn_Init(md.sys, r, dr),
	         "MoorDyn_Init failed for: " + c.input_file);

	// stepping (enforce fixed dt)
	double t = 0.0;
	double f[3] = { 0.0, 0.0, 0.0 };

	for (unsigned int i = 0; i < nsteps; ++i) {
		r[0] = x[static_cast<Eigen::Index>(i)];
		dr[0] = (x[static_cast<Eigen::Index>(i) + 1] -
		         x[static_cast<Eigen::Index>(i)]) /
		        dt0;

		double t_in = t;
		double dt_in = dt0;

		check_md(MoorDyn_Step(md.sys, r, dr, f, &t_in, &dt_in),
		         "MoorDyn_Step failed for: " + c.input_file);

		REQUIRE(std::abs(dt_in - dt0) <= 1e-12);
		t = t_in;
	}

	// close to flush outputs
	check_md(MoorDyn_Close(md.sys),
	         "MoorDyn_Close failed for: " + c.input_file);
	md.sys = nullptr;

	// read output mean tension
	const std::string out_path = line1_output_from_input(c.input_file);
	INFO("output=" << out_path);

	Eigen::VectorXd tension_mean_output;
	REQUIRE_NOTHROW(tension_mean_output = load_seg_te_column(out_path));

	const Eigen::Index n =
	    std::min<Eigen::Index>(tension_mean_output.size(), strain.size());
	REQUIRE(n >= 2);

	// preceding highest mean tension
	Eigen::VectorXd Tmax_mean(n);
	Tmax_mean[0] = kTmax0;
	for (Eigen::Index i = 1; i < n; ++i) {
		Tmax_mean[i] = std::max(Tmax_mean[i - 1], tension_mean_output[i]);
	}

	// analytical mean tension from WC + OWC
	Eigen::VectorXd tension_analytical(n);
	for (Eigen::Index i = 0; i < n; ++i) {
		tension_analytical[i] = find_mean_tension(strain[i],
		                                          tension_mean_output[i],
		                                          Tmax_mean[i],
		                                          owc_strains,
		                                          owc_tensions,
		                                          c.form);
	}

	const double denom = tension_analytical.squaredNorm();
	REQUIRE(denom > 0.0);

	const double l2_rel = std::sqrt(
	    (tension_analytical - tension_mean_output.head(n)).squaredNorm() /
	    denom);

	return l2_rel;
}

} // namespace

TEST_CASE("Syrope working curve regression: mean tension matches analytical WC",
          "[syrope][working-curve]")
{
	// shared OWC table for all cases
	const auto [owc_strains, owc_tensions] =
	    read_strain_tension_table("Mooring/syrope/owc.dat");

	const std::vector<WcCase> cases = {
		{ "Linear",
		  "Mooring/syrope/linear_wc_input.txt",
		  WorkingCurveForm::Linear,
		  1.37912e-02 },
		{ "Quadratic",
		  "Mooring/syrope/quadratic_wc_input.txt",
		  WorkingCurveForm::Quadratic,
		  1.41587e-02 },
		{ "Exponential",
		  "Mooring/syrope/exponential_wc_input.txt",
		  WorkingCurveForm::Exponential,
		  1.18597e-02 },
	};

	for (const auto& c : cases) {
		DYNAMIC_SECTION("Case: " << c.name)
		{
			const double l2_rel = run_case(c, owc_strains, owc_tensions);
			INFO("Relative L2 error = " << l2_rel);
			REQUIRE(l2_rel < kL2Tol);
		}
	}
}