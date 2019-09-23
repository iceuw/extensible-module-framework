#ifndef REFLECTOR_DETECTOR_
#define REFLECTOR_DETECTOR_
#include "function_block.hpp"
#include "reflector_utils.hpp"

namespace agv_robot
{
class ReflectorDetector : public FunctionBlock
{
private:
  ReflectorUtils ref_utils_;

public:
  ReflectorDetector(ConfigFile& cfg) {}
  ~ReflectorDetector() {}
  void Update(vector<Message*> inputs, vector<Message*> outputs);
};
EXPORT_INSTANCE(ReflectorDetector)
}
#endif