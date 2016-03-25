/*
 * VanillaEEConfigurationLoader.h
 */

#ifndef VANILLAEECONFIGURATIONLOADER_H
#define	VANILLAEECONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class VanillaEEConfigurationLoader : public ConfigurationLoader
{
	public:
		VanillaEEConfigurationLoader();
		~VanillaEEConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
