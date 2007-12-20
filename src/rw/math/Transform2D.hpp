#ifndef RW_MATH_TRANSFORM2D_HPP_
#define RW_MATH_TRANSFORM2D_HPP_

namespace rw {
namespace math {

/** @addtogroup math */
/*@{*/

	/**
	 * @brief A 3x3 homogeneous transform matrix @f$ \mathbf{T}\in SE(2) @f$
	 *
	 * @f$
	 * \mathbf{T} =
	 * \left[
	 *  \begin{array}{cc}
	 *  \mathbf{R} & \mathbf{d} \\
	 *  \begin{array}{cc}0 & 0\end{array} & 1
	 *  \end{array}
	 * \right]
	 * @f$
	 *
	 */

	class Transform2D
	{
	public:
		Transform2D();
		virtual ~Transform2D();
	};

}
}

#endif /*TRANSFORM2D_HPP_*/
