
/*
 * Jan. 19, 2017, David Z
 *
 * Use chi2_squared test in boost to compute chi2(m, belief)
 *
 * */


#ifndef CHI2_H
#define CHI2_H

#include <boost/math/distributions/chi_squared.hpp>

namespace utils
{
  double chi2(int dof, double alpha = 0.95)
  {
    if(dof > 0) 
      return boost::math::quantile(boost::math::chi_squared(dof), alpha); 
    else
    {
      std::cerr<<__FILE__<<" "<<__LINE__<<" dof = "<<dof<<" <= 0"<<std::endl;
    }
    return 0; 
  }
}


#endif
