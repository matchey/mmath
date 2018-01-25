
#ifndef MMATH_COMMON_ANGLES_H_
#define MMATH_COMMON_ANGLES_H_

namespace mmath
{
  /** \brief Convert an angle from radians to degrees
    * \param alpha the input angle (in radians)
    * \ingroup common
    */
  inline float 
  rad2deg (float alpha);

  /** \brief Convert an angle from degrees to radians
    * \param alpha the input angle (in degrees)
    * \ingroup common
    */
  inline float 
  deg2rad (float alpha);

  /** \brief Convert an angle from radians to degrees
    * \param alpha the input angle (in radians)
    * \ingroup common
    */
  inline double 
  rad2deg (double alpha);

  /** \brief Convert an angle from degrees to radians
    * \param alpha the input angle (in degrees)
    * \ingroup common
    */
  inline double 
  deg2rad (double alpha);

  /** \brief Normalize an angle to (-PI, PI]
    * \param alpha the input angle (in radians)
    * \ingroup common
    */
  inline float
  normAngle (float alpha);

  /** \brief Make an angle to acute(-PI/2, PI/2]
    * \param alpha the input angle (in radians)
    * \ingroup common
    */
  inline float
  acuteAngle (float alpha);
}

#include <mmath/common/impl/angles.hpp>

#endif  // MMATH_COMMON_ANGLES_H_

