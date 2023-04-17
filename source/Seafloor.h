/** @file Seafloor.h
 * C API for the moordyn::Seafloor object
 */

#ifndef SEAFLOOR_H
#define SEAFLOOR_H

#ifdef __cplusplus
extern "C"
{
#endif

	/// A 3D Seafloor instance
	typedef struct __MoorDynSeafloor* MoorDynSeafloor;

	/** @brief Get the depth of the seafloor at some x and y
	 * @param seafloor The Seafloor instance
	 * @param x The x coordinate
	 * @param y The y coordinate
	 * @param depth The output seafloor depth at that (x, y) point
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetDepthAt(MoorDynSeafloor seafloor,
	                               double x,
	                               double y,
	                               double* depth);

	/** @brief Get the average of depth of the seafloor
	 * This value is calculated as the average value of every depth point.
	 * If the rectilinear grid is not even, this average may not be the actual
	 * average depth of the surface the data describes.
	 * @param seafloor The Seafloor instance
	 * @param avgDepth The output average seafloor depth
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetAverageDepth(MoorDynSeafloor seafloor,
	                                    double* avgDepth);

	/** @brief The depth of the seafloor at the shallowest point
	 * @param seafloor The Seafloor instance
	 * @param minDepth The output minimum seafloor depth
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetMinDepth(MoorDynSeafloor seafloor, double* minDepth);

#ifdef __cplusplus
}
#endif

#endif // SEAFLOOR_H
