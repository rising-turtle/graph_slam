/*  
 * Oct. 17, 2016, David Z 
 *
 * parameters for VRO 
 *
 * */

#ifndef G2O_PARAMETERS
#define G2O_PARAMETERS

#include <string>
#include <cmath>

#define D2R(d) (((d)*M_PI)/180.)
#define R2D(r) (((r)*180.)/M_PI)

class CG2OParams
{
  public:
    // CParams();
    ~CG2OParams(); 
  
    int m_lookback_nodes;     // how many look back nodes  
    double m_small_translation;  // threshold for too small movement [meters]
    double m_small_rotation;     // threshold for too small rotation [degree]
    int m_optimize_step;      // every n nodes to optimize 
    std::string m_output_dir;    // where to save the output result
    double m_initial_pitch;   // pitch between camera and the floor, degree 

  static CG2OParams* Instance(); 

  private:
    CG2OParams();
    static CG2OParams* mp_instance;
    // TODO: add siftgpu_edge|contrast|_threshold, sift_|octave|contrast_threshold 
};

#endif
