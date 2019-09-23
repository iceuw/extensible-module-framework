#include "system.hpp"
#include "io.hpp"

using namespace agv_robot;

int main(int argc, char** argv)
{
	string cfg_name = "localization.ini";
	ConfigFile cfg(cfg_name);
	string prototxt = cfg.value("prototxt", "filename", "test.prototxt");
	if (argc > 1)
	{
		cfg_name = argv[1];
	}
	stdmsg::Net* net = new stdmsg::Net();
	ReadProtoFromTextFile(prototxt, net);
	System sys(cfg, *net);
	sys.Run();
	return 0;
}