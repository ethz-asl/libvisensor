/*
 * visensor_config.hpp
 *
 *  Created on: Feb 19, 2014
 *      Author: skybotix
 */

#ifndef VISENSOR_CONFIG_HPP_
#define VISENSOR_CONFIG_HPP_

//Control symbol exports
#ifdef VISENSOR_EXPORT
#   include <config/compiler.hpp>
#else
#   define DSO_EXPORT
#   define DSO_IMPORT
#endif


#endif /* VISENSOR_CONFIG_HPP_ */
