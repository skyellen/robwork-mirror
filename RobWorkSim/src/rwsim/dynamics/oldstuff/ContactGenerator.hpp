/*
 * ContactGenerator.hpp
 *
 *  Created on: 23-10-2008
 *      Author: jimali
 */

#ifndef CONTACTGENERATOR_HPP_
#define CONTACTGENERATOR_HPP_


namespace rwsim {
namespace dynamics {

	class ContactGenerator {
	public:

		ContactGenerator(){};

		virtual ~ContactGenerator(){};

		/**
		 * @brief
		 * @param points
		 * @param manifolds
		 */
		void generateContactManifolds(
			std::vector<ContactPoint*>& points,
			std::vector<ContactManifold*>& manifolds);

	};
}
}
#endif /* CONTACTGENERATOR_HPP_ */
