
#include "gt_parameter.h"

CGTParams* CGTParams::mp_instance = NULL; 
CGTParams* CGTParams::Instance()
{
  if(mp_instance == NULL)
  {
    mp_instance = new CGTParams(); 
  }
  return mp_instance;
}

CGTParams::CGTParams(): 
  m_lookback_nodes(7),
  m_small_translation(0.05), 
  m_small_rotation(3),
  m_large_translation(2.), 
  m_large_rotation(20),
  m_optimize_step(10), 
  m_output_dir("./"), 
  m_initial_pitch(0),
  m_vro_result("vro_results.log")
{}
CGTParams::~CGTParams(){}
