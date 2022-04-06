/*
Copyright 2020 Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include "Modules/ModuleManager.h"


class FPowerIKToolModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
