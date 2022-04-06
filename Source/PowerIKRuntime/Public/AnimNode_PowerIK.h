/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include <string>
#include "CoreMinimal.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"

#include "PowerIKRuntime/Public/PowerIK_UnrealCore.h"

#include "AnimNode_PowerIK.generated.h"

DECLARE_CYCLE_STAT(TEXT("PowerIK Total"), STAT_PowerIK_Eval, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("PowerIK Solve"), STAT_PowerIK_Solve, STATGROUP_Anim);


USTRUCT(BlueprintInternalUseOnly)
struct POWERIKRUNTIME_API FAnimNode_PowerIK : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Name of joint that acts as the root of the solve. All effectors must be on children of this bone. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Rig, meta = (PinShownByDefault))
	FName CharacterRoot = "pelvis";

	/** How much to rotate root towards neighboring effectors. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	float RootRotationMultiplier = 1.0f;

	/** Apply inertial damping to Character Root bone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	FPowerIKBodyInertia BodyInertia;

	/** List of custom directions for bones to bend in. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKBoneBendDirection> BendDirections;

	/** List of excluded bones. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKExcludedBone> ExcludedBones;

	/** List of rotation limits for joints. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKBoneLimit> JointLimits;
	
	/** List of effectors to pull the rig. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Effectors, meta = (PinShownByDefault))
	TArray<FPowerIKEffector> Effectors;

	/** Global multiplier applied to maximum speed effectors can react to input position when smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Effectors, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float SmoothingMaxSpeedMultiplier = 1.0f;

	/** Global multiplier applied to maximum speed effectors can react to input position when smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Effectors, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float SmoothingMaxDistanceMultiplier = 1.0f;

	/** Center of Gravity Constraint, applied to Character Root bone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	FPowerIKCenterOfGravity CenterOfGravityConstraint;

	/** Number of iterations to improve squashing poses. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "1", ClampMax = "200", UIMin = "1", UIMax = "80"))
	int32 MaxSquashIterations = 6;

	/** Number of iterations to improve stretching poses. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "1", ClampMax = "200", UIMin = "1", UIMax = "80"))
	int32 MaxStretchIterations = 6;

	/** Number of iterations to cleanup final pose. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "1", ClampMax = "200", UIMin = "1", UIMax = "80"))
	int32 MaxFinalIterations = 3;

	/** If true, bone lengths will be updated each frame based on the input pose. This allows bone chains to 'stretch' but has a small additional cost to support.*/
	UPROPERTY(EditAnywhere, Category = SolverSettings)
	bool AllowBoneTranslation = false;

	/** Global alpha to blend effector of solver on/off from 0 to 1. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"), meta = (PinShownByDefault))
	float SolverAlpha = 1.0f;

	/** Adjust size of debug gizmos. */
	UPROPERTY(EditAnywhere, Category = Debug, meta = (ClampMin = "1", ClampMax = "2000", UIMin = "1", UIMax = "100"))
	float DebugDrawSize = 20.0f;

	UWorld* World;

public:
	// FAnimNode_Base interface
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual void CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)  override;
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
#if WITH_EDITOR
	void ConditionalDebugDraw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* MeshComp) const;
#endif
	// End of FAnimNode_SkeletalControlBase interface

private:

	FPowerIKCore Core;
};
