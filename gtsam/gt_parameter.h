/*  
 * Oct. 2, 2016, David Z 
 *
 * parameters for VRO 
 *
 * */

#ifndef GT_PARAMETERS
#define GT_PARAMETERS

#include <string>
#include <cmath>

#define D2R(d) (((d)*M_PI)/180.)
#define R2D(r) (((r)*180.)/M_PI)

class CGTParams
{
  public:
    // CParams();
    ~CGTParams(); 
  
    int m_lookback_nodes;     // how many look back nodes  
    double m_small_translation;  // threshold for too small movement [meters]
    double m_small_rotation;     // threshold for too small rotation [degree]
    double m_large_translation;  // threshold for too small movement [meters]
    double m_large_rotation;     // threshold for too small rotation [degree]
    int m_optimize_step;      // every n nodes to optimize 
    std::string m_output_dir;    // where to save the output result
    double m_initial_pitch;   // pitch between camera and the floor, degree 
    std::string m_vro_result; // where to save vro result 

  static CGTParams* Instance(); 

  private:
    CGTParams();
    static CGTParams* mp_instance;
    // TODO: add siftgpu_edge|contrast|_threshold, sift_|octave|contrast_threshold 
};

#endif
