// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "PowerIK_UnrealCore.h"
#include "Units/RigUnit.h"
#include "RigUnit_PowerIK.generated.h"

/**
 * Power IK Solver Control Rig Node
 */
USTRUCT(meta=(DisplayName="PowerIK Solver", Category = "Hierarchy", Keywords="IK,Solver,Power"))
struct FRigUnit_PowerIK : public FRigUnitMutable
{
	GENERATED_BODY()

	RIGVM_METHOD()
	virtual void Execute(const FRigUnitContext& Context) override;

	/** Name of joint that acts as the root of the solve. All effectors must be on children of this bone. */
	UPROPERTY(meta = (Input, Constant, CustomWidget = "BoneName"))
	FName CharacterRoot = "pelvis";

	/** How much to rotate root towards neighboring effectors. */
	UPROPERTY(meta = (Input))
	float RootRotationMultiplier = 1.0f;

	/** List of effectors to pull the rig. */
	UPROPERTY(meta = (Input))
	TArray<FPowerIKEffector> Effectors;

	/** List of custom directions for bones to bend in. */
	UPROPERTY(meta = (Input, Constant))
	TArray<FPowerIKBoneBendDirection> BendDirections;

	/** List of excluded bones. */
	UPROPERTY(meta = (Input, Constant, CustomWidget = "BoneName"))
	TArray<FPowerIKExcludedBone> ExcludedBones;

	/** List of rotation limits for joints. */
	UPROPERTY(meta = (Input))
	TArray<FPowerIKBoneLimit> JointLimits;

	/** Center of Gravity Constraint, applied to Character Root bone. */
	UPROPERTY(meta = (Input))
	FPowerIKCenterOfGravity CenterOfGravityConstraint;

	/** Apply inertial damping to Character body. */
	UPROPERTY(meta = (Input))
	FPowerIKBodyInertia Inertia;

	/** Number of iterations to improve squashing poses. */
	UPROPERTY(meta = (Input))
	int32 MaxSquashIterations = 6;

	/** Number of iterations to improve stretching poses. */
	UPROPERTY(meta = (Input))
	int32 MaxStretchIterations = 6;

	/** Number of iterations to improve final pose. */
	UPROPERTY(meta = (Input))
	int32 MaxFinalIterations = 6;

	/** If true, bone lengths will be updated each frame based on the input pose. This allows bone chains to 'stretch' but has a small additional cost to support.*/
	UPROPERTY(meta = (Input))
	bool AllowBoneTranslation = false;

	/** Global alpha to blend effector of solver on/off from 0 to 1. */
	UPROPERTY(meta = (Input))
	float SolverAlpha = 1.0f;

	UPROPERTY(transient)
	FPowerIKCore Core;
};

