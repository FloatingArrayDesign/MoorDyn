#pragma once

#include <vector>
#include "../Misc.hpp"
namespace moordyn {
/** \defgroup interpolation Interpolation utilities
 *  @{
 */

/** @brief One-dimensional linear interpolation factor
 * @param xp The points where data is available
 * @param i0 The starting index to look for the upper bound
 * @param x The evaluation point
 * @param f The interpolation factor
 * @return The index of the upper bound
 */
template<typename T>
inline unsigned int
interp_factor(const std::vector<T>& xp, unsigned int i0, const T& x, T& f)
{
	if (xp.size() == 1) {
		f = 0.0;
		return 0;
	}

	if (i0 == 0)
		i0++;
	if (i0 > xp.size() - 1)
		i0 = static_cast<unsigned int>(xp.size() - 1);

	if (x <= xp[i0 - 1]) {
		f = 0.0;
		return i0;
	}
	if (x >= xp.back()) {
		f = 1.0;
		return static_cast<unsigned int>(xp.size() - 1);
	}

	for (unsigned i = i0; i < xp.size(); i++) {
		if (x <= xp[i]) {
			f = (x - xp[i - 1]) / (xp[i] - xp[i - 1]);
			return i;
		}
	}

	// Just to avoid the compiler warnings. This point is actually never reached
	assert(false);
	f = 1.0;
	return static_cast<unsigned int>(xp.size() - 1);
}

/**
 * @brief Basic linear interpolation
 *
 * Implemented as `a * (1 - factor) + (b * factor)`
 *
 * @tparam T Type being interpolated between
 * @tparam F Type of interpolation factor, probably double or float
 * @param a staring value, lerp(a, b, 0) = a
 * @param b ending value, lerp(a, b, 1) = b
 * @param factor Interpolation factor, should be between 0.0 and 1.0
 * @return T
 */
template<typename T, typename F>
T
lerp(const T& a, const T& b, const F& factor)
{
	return a * (1 - factor) + (b * factor);
}
/** @brief One-dimensional linear interpolation factor
 *
 * This function is equivalent to calling interp_factor(xp, 1, x, f)
 * @param xp The points where data is available
 * @param x The evaluation point
 * @param f The interpolation factor
 * @return The index of the upper bound
 */
template<typename T>
inline unsigned int
interp_factor(const std::vector<T>& xp, const T& x, T& f)
{
	return interp_factor(xp, 1, x, f);
}

/** @brief One-dimensional linear interpolation
 *
 * For monotonically increasing sample points.
 * @param xp The points where data is available
 * @param yp The data values
 * @param x The evaluation points
 * @param y The interpolated values. The vector shall be already initialized
 * with at least the same number of components than \p x
 */
template<typename Tx, typename Ty>
inline void
interp(const std::vector<Tx>& xp,
       const std::vector<Ty>& yp,
       const std::vector<Tx>& x,
       std::vector<Ty>& y)
{
	if (yp.size() == 1) {
		y[0] = yp[0];
		return;
	}

	real f;
	unsigned int j = 1;
	for (unsigned int i = 0; i < x.size(); i++) {
		j = interp_factor(xp, j, x[i], f);
		y[i] = yp[j - 1] + f * (yp[j] - yp[j - 1]);
	}
}

/** @brief One-dimensional linear interpolation
 * @param xp The points where data is available
 * @param yp The data values
 * @param x The evaluation point
 * @return The interpolated value
 */
template<typename Tx, typename Ty>
inline Ty
interp(const std::vector<Tx>& xp, const std::vector<Ty>& yp, Tx x)
{
	if (yp.size() == 1) {
		return yp[0];
	}

	real f;
	const auto j = interp_factor(xp, 1, x, f);
	return yp[j - 1] + f * (yp[j] - yp[j - 1]);
}

/** @brief Bilinear filter
 * @param values The available data
 * @param i The upper bound index in the x direction
 * @param j The upper bound index in the y direction
 * @param fx The linear interplation factor in the x direction
 * @param fy The linear interplation factor in the y direction
 * @return The linearly interpolated value
 * @see interp_factor
 */
template<typename T>
inline T
interp2(const std::vector<std::vector<T>>& values,
        unsigned int i,
        unsigned int j,
        T fx,
        T fy)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;

	T c00 = values[i0][j0];
	T c01 = values[i0][j];
	T c10 = values[i][j0];
	T c11 = values[i][j];

	T c0 = c00 * (1. - fx) + c10 * fx;
	T c1 = c01 * (1. - fx) + c11 * fx;

	return c0 * (1 - fy) + c1 * fy;
}

/** @brief Trilinear filter
 * @param values The available data
 * @param i The upper bound index in the x direction
 * @param j The upper bound index in the y direction
 * @param k The upper bound index in the z direction
 * @param fx The linear interplation factor in the x direction
 * @param fy The linear interplation factor in the y direction
 * @param fz The linear interplation factor in the z direction
 * @return The linearly interpolated value
 * @see interp_factor
 */
template<typename T>
inline T
interp3(const std::vector<std::vector<std::vector<T>>>& values,
        unsigned int i,
        unsigned int j,
        unsigned int k,
        T fx,
        T fy,
        T fz)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;
	unsigned int k0 = k > 0 ? k - 1 : 0;

	T c000 = values[i0][j0][k0];
	T c001 = values[i0][j0][k];
	T c010 = values[i0][j][k0];
	T c011 = values[i0][j][k];
	T c100 = values[i][j0][k0];
	T c101 = values[i][j0][k];
	T c110 = values[i][j][k0];
	T c111 = values[i][j][k];

	T c00 = c000 * (1. - fx) + c100 * fx;
	T c01 = c001 * (1. - fx) + c101 * fx;
	T c10 = c010 * (1. - fx) + c110 * fx;
	T c11 = c011 * (1. - fx) + c111 * fx;

	T c0 = c00 * (1. - fy) + c10 * fy;
	T c1 = c01 * (1. - fy) + c11 * fy;

	return c0 * (1 - fz) + c1 * fz;
}

/** @brief Quadrilinear filter
 * @param values The available data
 * @param i The upper bound index in the x direction
 * @param j The upper bound index in the y direction
 * @param k The upper bound index in the z direction
 * @param w The upper bound index in the w direction
 * @param fx The linear interplation factor in the x direction
 * @param fy The linear interplation factor in the y direction
 * @param fz The linear interplation factor in the z direction
 * @param fw The linear interplation factor in the w direction
 * @return The linearly interpolated value
 * @see interp_factor
 */
template<typename T>
inline T
interp4(const std::vector<std::vector<std::vector<std::vector<T>>>>& values,
        unsigned int i,
        unsigned int j,
        unsigned int k,
        unsigned int w,
        T fx,
        T fy,
        T fz,
        T fw)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;
	unsigned int k0 = k > 0 ? k - 1 : 0;
	unsigned int w0 = w > 0 ? w - 1 : 0;

	T c000 = values[i0][j0][k0][w0] * (1. - fw) + values[i0][j0][k0][w] * fw;
	T c001 = values[i0][j0][k][w0] * (1. - fw) + values[i0][j0][k][w] * fw;
	T c010 = values[i0][j][k0][w0] * (1. - fw) + values[i0][j][k0][w] * fw;
	T c011 = values[i0][j][k][w0] * (1. - fw) + values[i0][j][k][w] * fw;
	T c100 = values[i][j0][k0][w0] * (1. - fw) + values[i][j0][k0][w] * fw;
	T c101 = values[i][j0][k][w0] * (1. - fw) + values[i][j0][k][w] * fw;
	T c110 = values[i][j][k0][w0] * (1. - fw) + values[i][j][k0][w] * fw;
	T c111 = values[i][j][k][w0] * (1. - fw) + values[i][j][k][w] * fw;

	T c00 = c000 * (1. - fx) + c100 * fx;
	T c01 = c001 * (1. - fx) + c101 * fx;
	T c10 = c010 * (1. - fx) + c110 * fx;
	T c11 = c011 * (1. - fx) + c111 * fx;

	T c0 = c00 * (1. - fy) + c10 * fy;
	T c1 = c01 * (1. - fy) + c11 * fy;

	return c0 * (1 - fz) + c1 * fz;
}

template<typename T, typename R>
inline T
interp4Vec(const std::vector<std::vector<std::vector<std::vector<T>>>>& values,
           unsigned int i,
           unsigned int j,
           unsigned int k,
           unsigned int w,
           R fx,
           R fy,
           R fz,
           R fw)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;
	unsigned int k0 = k > 0 ? k - 1 : 0;
	unsigned int w0 = w > 0 ? w - 1 : 0;

	T c000 = values[i0][j0][k0][w0] * (1. - fw) + values[i0][j0][k0][w] * fw;
	T c001 = values[i0][j0][k][w0] * (1. - fw) + values[i0][j0][k][w] * fw;
	T c010 = values[i0][j][k0][w0] * (1. - fw) + values[i0][j][k0][w] * fw;
	T c011 = values[i0][j][k][w0] * (1. - fw) + values[i0][j][k][w] * fw;
	T c100 = values[i][j0][k0][w0] * (1. - fw) + values[i][j0][k0][w] * fw;
	T c101 = values[i][j0][k][w0] * (1. - fw) + values[i][j0][k][w] * fw;
	T c110 = values[i][j][k0][w0] * (1. - fw) + values[i][j][k0][w] * fw;
	T c111 = values[i][j][k][w0] * (1. - fw) + values[i][j][k][w] * fw;

	T c00 = c000 * (1. - fx) + c100 * fx;
	T c01 = c001 * (1. - fx) + c101 * fx;
	T c10 = c010 * (1. - fx) + c110 * fx;
	T c11 = c011 * (1. - fx) + c111 * fx;

	T c0 = c00 * (1. - fy) + c10 * fy;
	T c1 = c01 * (1. - fy) + c11 * fy;

	return c0 * (1 - fz) + c1 * fz;
}

/**
 * @}
 */
}
