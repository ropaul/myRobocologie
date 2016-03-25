/*
 * vanilla_EEConfigurationLoader.h
 */

#ifndef VANILLA_EECONFIGURATIONLOADER_H
#define	VANILLA_EECONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class vanilla_EEConfigurationLoader : public ConfigurationLoader
{
	public:
		vanilla_EEConfigurationLoader();
		~vanilla_EEConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
