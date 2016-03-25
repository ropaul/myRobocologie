#if defined PRJ_VANILLA_EE || !defined MODULAR

#include "Config/vanilla_EEConfigurationLoader.h"

#include "vanilla_EE/include/vanilla_EEWorldObserver.h"
#include "vanilla_EE/include/vanilla_EEAgentObserver.h"
#include "vanilla_EE/include/vanilla_EEController.h"

#include "WorldModels/RobotWorldModel.h"


vanilla_EEConfigurationLoader::vanilla_EEConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

vanilla_EEConfigurationLoader::~vanilla_EEConfigurationLoader()
{
	//nothing to do
}

WorldObserver* vanilla_EEConfigurationLoader::make_WorldObserver(World* wm)
{
	return new vanilla_EEWorldObserver(wm);
}

RobotWorldModel* vanilla_EEConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* vanilla_EEConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new vanilla_EEAgentObserver(wm);
}

Controller* vanilla_EEConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new vanilla_EEController(wm);
}


#endif
