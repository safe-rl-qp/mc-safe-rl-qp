#include "Go2Control.h"

#include <mc_rtc/config.h>

#include <fstream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace
{

/* Checking the existence of the file */
/* Return value: true if the file exists, false otherwise */
bool file_exists(const std::string& str)
{
  std::ifstream fs(str);
  return fs.is_open();
}

} // namespace

/* Main function of the interface */
int main(int argc, char * argv[])
{
  /* Set command line arguments options */
  /* Usage example: MCControlUnitree2 -h simulation -f @ETC_PATH@/mc_unitree/mc_rtc_xxxxx.yaml */
  std::string conf_file;
  std::string network;
  po::options_description desc(std::string("MCControlGo2 options"));
  
  // Get the configuration file path dedicated to this program
  std::string check_file = mc_unitree::CONFIGURATION_FILE;
  if(!file_exists(check_file))
  {
    check_file = "";
  }
  
  // clang-format off
  desc.add_options()
    ("help", "display help message")
    ("networ,n", po::value<std::string>(&network)->default_value("eth0"), "name of network adaptor")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(check_file), "configuration file");
  // clang-format on

  /* Parse command line arguments */
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
  po::notify(vm);
  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  mc_rtc::log::info("[mc_unitree] Reading additional configuration from {}", conf_file);

  /* Create global controller */
  mc_control::MCGlobalController g_controller(conf_file, nullptr);

  /* Check that the interface can work with the main controller robot */
  std::string module_name;
  module_name.resize(mc_unitree::ROBOT_NAME.size());
  std::transform(mc_unitree::ROBOT_NAME.begin(), mc_unitree::ROBOT_NAME.end(), module_name.begin(), ::tolower);
  if(g_controller.robot().name() != module_name)
  {
    mc_rtc::log::error(
        "[mc_unitree] This program can only handle '" + mc_unitree::ROBOT_NAME + "' at the moment");
    return 1;
  }
  
  /* Create MCControlUnitree2 interface */
  mc_unitree::MCControlUnitree2<mc_unitree::Go2Control,
                                mc_unitree::Go2SensorInfo,
                                mc_unitree::Go2CommandData,
                                mc_unitree::Go2ConfigParameter> mc_control_unitree(g_controller,
                                                                                   network);

  mc_rtc::log::info("[mc_unitree] Terminated");

  return 0;
}
