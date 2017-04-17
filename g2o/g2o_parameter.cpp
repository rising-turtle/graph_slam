
#include "g2o_parameter.h"

CG2OParams* CG2OParams::mp_instance = NULL; 
CG2OParams* CG2OParams::Instance()
{
  if(mp_instance == NULL)
  {
    mp_instance = new CG2OParams(); 
  }
  return mp_instance;
}

CG2OParams::CG2OParams(): 
  m_lookback_nodes(7),
  m_small_translation(0.05), 
  m_small_rotation(3),
  m_optimize_step(10), 
  m_output_dir("./"), 
  m_initial_pitch(0)
{}
CG2OParams::~CG2OParams(){}
