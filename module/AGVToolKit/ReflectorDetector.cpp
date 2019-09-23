#include "ReflectorDetector.hpp"

namespace agv_robot
{
  void ReflectorDetector::Update(vector<Message*> inputs, vector<Message*> outputs)
  {
    GOOGLE_CHECK_GE(inputs.size(), 1) << "The reflector detector needs more than "
      << "one scan input, but actually recevied " << inputs.size() << "scan inputs";

    for (Message* input : inputs)
    {
      //GOOGLE_CHECK_EQ(typeid(input), typeid(stdmsg::Laser_Scan*)) << "The reflector "
      //  << "detector received the msg that is't Laser_Scan";

        stdmsg::Laser_Scan* scan = (stdmsg::Laser_Scan*)inputs[0];
        ref_utils_.GetOneScan(scan);

#undef scan
    }
  }
}  // namespace agv_robot