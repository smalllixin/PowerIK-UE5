/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#include "PowerIKRuntimeModule.h"

#define LOCTEXT_NAMESPACE "FPowerIKRuntimeModule"

void FPowerIKRuntimeModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FPowerIKRuntimeModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FPowerIKRuntimeModule, PowerIKRuntime)