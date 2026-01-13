/** @file syrope.cpp
 * @brief Test case for Syrope model.
 * Compares MoorDyn simulation output against analytical expectations from given original working curve
 * and working curves.
 */

#include "MoorDyn2.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double G      = 9.80665;
constexpr double MBL    = 885.0e3 * G;
constexpr double TMAX_0 = 0.15 * MBL;
constexpr double K1     = 1.25e8;

// Linear interpolation on monotone-increasing xdata, clamped to endpoints.
double interpolate_clamped(double x,
                           const Eigen::VectorXd& xdata,
                           const Eigen::VectorXd& ydata)
{
    if (xdata.size() != ydata.size())
        throw std::invalid_argument("interpolate_clamped: size mismatch");
    if (xdata.size() < 2)
        throw std::invalid_argument("interpolate_clamped: need >= 2 points");

    if (x <= xdata[0]) return ydata[0];
    if (x >= xdata[xdata.size() - 1]) return ydata[ydata.size() - 1];

    Eigen::Index lo = 0;
    Eigen::Index hi = xdata.size() - 1;
    while (hi - lo > 1) {
        const Eigen::Index mid = lo + (hi - lo) / 2;
        if (xdata[mid] <= x) lo = mid;
        else hi = mid;
    }

    const double x0 = xdata[lo], x1 = xdata[hi];
    const double y0 = ydata[lo], y1 = ydata[hi];
    const double dx = x1 - x0;

    if (std::abs(dx) <= std::numeric_limits<double>::epsilon())
        return 0.5 * (y0 + y1);

    const double t = (x - x0) / dx;
    return y0 + t * (y1 - y0);
}

double find_mean_tension(double strain,
                         double Tmean,
                         double Tmax,
                         const Eigen::VectorXd& owc_strains,
                         const Eigen::VectorXd& owc_tensions)
{
    if (Tmean < Tmax) {
        const double eps_max = interpolate_clamped(Tmax, owc_tensions, owc_strains);
        const double eps_min = eps_max - Tmax / K1;
        return Tmax * (strain - eps_min) / (eps_max - eps_min);
    }
    return interpolate_clamped(strain, owc_strains, owc_tensions);
}

bool read_three_whitespace_columns_skip_header(const std::string& path,
                                              int header_lines,
                                              std::vector<double>& c0,
                                              std::vector<double>& c1,
                                              std::vector<double>& c2)
{
    std::ifstream in(path);
    if (!in.is_open()) return false;

    std::string line;
    for (int i = 0; i < header_lines; ++i) {
        if (!std::getline(in, line)) return false;
    }

    while (std::getline(in, line)) {
        std::istringstream iss(line);
        double a = 0.0, b = 0.0, d = 0.0;
        if (!(iss >> a >> b >> d)) continue;
        c0.push_back(a);
        c1.push_back(b);
        c2.push_back(d);
    }
    return true;
}

bool read_two_whitespace_columns(const std::string& path,
                                std::vector<double>& c0,
                                std::vector<double>& c1)
{
    std::ifstream in(path);
    if (!in.is_open()) return false;

    std::string line;
    while (std::getline(in, line)) {
        std::istringstream iss(line);
        double a = 0.0, b = 0.0;
        if (!(iss >> a >> b)) continue;
        c0.push_back(a);
        c1.push_back(b);
    }
    return true;
}

} // namespace

TEST_CASE("Syrope: simulation runs and working-curve postprocessing is consistent for slow loading")
{
    // --- Run MoorDyn case ---
    MoorDyn system = MoorDyn_Create("Mooring/syrope/slow_loading.txt");
    REQUIRE(system);

    unsigned int ndof = 0;
    REQUIRE(MoorDyn_NCoupledDOF(system, &ndof) == MOORDYN_SUCCESS);
    REQUIRE(ndof == 3);

    double r[3]  = {0.0, 0.0, 0.0};
    double dr[3] = {0.0, 0.0, 0.0};

    MoorDynPoint anchor   = MoorDyn_GetPoint(system, 1);
    MoorDynPoint fairlead = MoorDyn_GetPoint(system, 2);
    MoorDynLine  line     = MoorDyn_GetLine(system, 1);
    REQUIRE(anchor);
    REQUIRE(fairlead);
    REQUIRE(line);

    double l0 = 0.0;
    REQUIRE(MoorDyn_GetLineUnstretchedLength(line, &l0) == MOORDYN_SUCCESS);

    REQUIRE(MoorDyn_GetPointPos(fairlead, r) == MOORDYN_SUCCESS);
    std::fill(dr, dr + 3, 0.0);
    REQUIRE(MoorDyn_Init(system, r, dr) == MOORDYN_SUCCESS);

    const double tdur = 3600.0 * 3.0;  // 3 hours for one loading segment
    const double Tdur = tdur * 6.0;    // six loading segments
    double dt = 10.0;
    const unsigned int nsteps = static_cast<unsigned int>(Tdur / dt);

    Eigen::VectorXd times   = Eigen::VectorXd::LinSpaced(static_cast<Eigen::Index>(nsteps) + 1, 0.0, Tdur);
    Eigen::VectorXd strains = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(nsteps) + 1);

    const double eps_0 = 1.37912e-02;
    for (unsigned int i = 0; i <= nsteps; ++i) {
        const double tt = times[static_cast<Eigen::Index>(i)];
        double eps = eps_0;

        if (tt < tdur)                  eps = eps_0;
        else if (tt < 2.0 * tdur)       eps = eps_0 + 2.0 * eps_0 * (tt - tdur) / tdur;
        else if (tt < 3.0 * tdur)       eps = eps_0 + 2.0 * eps_0;
        else if (tt < 4.0 * tdur)       eps = eps_0 + 2.0 * eps_0 - 1.0 * eps_0 * (tt - 3.0 * tdur) / tdur;
        else if (tt < 5.0 * tdur)       eps = eps_0 + 1.0 * eps_0;
        else                            eps = eps_0 + 1.0 * eps_0 + 1.5 * eps_0 * (tt - 5.0 * tdur) / tdur;

        strains[static_cast<Eigen::Index>(i)] = eps;
    }

    double t = 0.0;
    double flat[3] = {0.0, 0.0, 0.0};

    for (unsigned int i = 0; i < nsteps; ++i) {
        r[0] = 1.0 + strains[static_cast<Eigen::Index>(i)];

        double t_in  = times[static_cast<Eigen::Index>(i)];
        double dt_in = dt;

        INFO("Step i=" << i << " t=" << t_in << " dt=" << dt_in << " r[0]=" << r[0]);
        REQUIRE(MoorDyn_Step(system, r, dr, flat, &t_in, &dt_in) == MOORDYN_SUCCESS);

        dt = dt_in;
    }

    REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);

    // --- Read MoorDyn output ---
    std::vector<double> timedata;
    std::vector<double> tensiondata; // Mean tension instead of instant tension, 
                                     // MoorDyn_GetLineNodeTen() returns instant tension 
                                     // (sum of the axial stiffness plus the internal damping)
    std::vector<double> straindata;
    REQUIRE(read_three_whitespace_columns_skip_header("Mooring/syrope/slow_loading_Line1.out",
                                                     /*header_lines=*/2,
                                                     timedata, tensiondata, straindata));

    REQUIRE(timedata.size() >= 2);
    REQUIRE(tensiondata.size() == timedata.size());
    REQUIRE(straindata.size() == timedata.size());

    const Eigen::Index n_csv = static_cast<Eigen::Index>(timedata.size());
    const Eigen::VectorXd csv_time =
        Eigen::Map<const Eigen::VectorXd>(timedata.data(), n_csv);
    const Eigen::VectorXd csv_tension =
        Eigen::Map<const Eigen::VectorXd>(tensiondata.data(), n_csv);
    const Eigen::VectorXd csv_strain =
        Eigen::Map<const Eigen::VectorXd>(straindata.data(), n_csv);

    const Eigen::Index n_use = std::min<Eigen::Index>(n_csv, strains.size());
    REQUIRE(n_use >= 2);

    // --- Read OWC (strain, tension) ---
    std::vector<double> owc_strain_data, owc_tension_data;
    REQUIRE(read_two_whitespace_columns("Mooring/syrope/owc.dat", owc_strain_data, owc_tension_data));
    REQUIRE(owc_strain_data.size() >= 2);
    REQUIRE(owc_strain_data.size() == owc_tension_data.size());

    const Eigen::VectorXd owc_strains =
        Eigen::Map<const Eigen::VectorXd>(owc_strain_data.data(),
                                          static_cast<Eigen::Index>(owc_strain_data.size()));
    const Eigen::VectorXd owc_tensions =
        Eigen::Map<const Eigen::VectorXd>(owc_tension_data.data(),
                                          static_cast<Eigen::Index>(owc_tension_data.size()));

    // --- Post-process: Tmax and analytical tension ---
    Eigen::VectorXd Tmax = Eigen::VectorXd::Zero(n_use);
    Tmax[0] = TMAX_0;
    for (Eigen::Index i = 1; i < n_use; ++i) {
        Tmax[i] = std::max(Tmax[i - 1], csv_tension[i]);
    }

    Eigen::VectorXd anal_tension = Eigen::VectorXd::Zero(n_use);
    for (Eigen::Index i = 0; i < n_use; ++i) {
        anal_tension[i] = find_mean_tension(strains[i], csv_tension[i], Tmax[i], owc_strains, owc_tensions);
    }

    // Relative L2 error
    const Eigen::VectorXd abs_err = (csv_tension.head(n_use) - anal_tension).cwiseAbs();
    const double denom = std::max(anal_tension.squaredNorm(), 1e-30);
    const double rel_l2 = std::sqrt(abs_err.squaredNorm() / denom);

    REQUIRE(std::isfinite(rel_l2));
    
    // For slow loading, expect <1% relative L2 error
    REQUIRE(rel_l2 < 0.01);
}