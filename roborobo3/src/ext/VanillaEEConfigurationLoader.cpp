#if defined PRJ_VANILLAEE || !defined MODULAR

#include "Config/VanillaEEConfigurationLoader.h"

#include "VanillaEE/include/VanillaEEWorldObserver.h"
#include "VanillaEE/include/VanillaEEAgentObserver.h"
#include "VanillaEE/include/VanillaEEController.h"

#include "WorldModels/RobotWorldModel.h"


VanillaEEConfigurationLoader::VanillaEEConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

VanillaEEConfigurationLoader::~VanillaEEConfigurationLoader()
{
	//nothing to do
}

WorldObserver* VanillaEEConfigurationLoader::make_WorldObserver(World* wm)
{
	return new VanillaEEWorldObserver(wm);
}

RobotWorldModel* VanillaEEConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* VanillaEEConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new VanillaEEAgentObserver(wm);
}

Controller* VanillaEEConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new VanillaEEController(wm);
}


#endif
