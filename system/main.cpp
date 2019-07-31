#include "system.hpp"
#include "io.hpp"

using namespace agv_robot;

int main(int argc, char** argv)
{
	string cfg_name = "localization.ini";
	if (argc > 1)
	{
		cfg_name = argv[1];
	}
	stdmsg::Net* net = new stdmsg::Net();
	ReadProtoFromTextFile("test.prototxt", net);
	ConfigFile cfg(cfg_name);
	System sys(cfg, *net);
	sys.Run();
}