/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include <string>
#include "CoreMinimal.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "Curves/CurveFloat.h"
#include "UObject/ConstructorHelpers.h"

#include "PowerIKRuntime/Public/PowerIK_UnrealCore.h"

#include "AnimNode_PowerIK_Walk.generated.h"

DECLARE_CYCLE_STAT(TEXT("PowerIK Walk Total"), STAT_PowerIK_Walk_Eval, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("PowerIK Walk Solve"), STAT_PowerIK_Walk_Solve, STATGROUP_Anim);


/* TODO
// do second raycast to snap corrected foot position to ground height
// clamp max stride length (record root transform at last planted location)
*/

USTRUCT(BlueprintInternalUseOnly)
struct POWERIKRUNTIME_API FPlaneConstraint
{
	GENERATED_USTRUCT_BODY()

	bool Active = false;
	FVector Normal;
};

USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKWalkingLimb
{
	GENERATED_USTRUCT_BODY()

	/** Name of the bone at the start of this limb. This is the origin of the raycast. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Limb)
	FName StartBone;
	int32 StartBoneRefIndex;
	int32 StartBoneCompactIndex;

	/** Name of foot bone at end of the limb. This will be tip of IK effect. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Limb)
	FName EndBone;
	int32 EndBoneRefIndex;
	int32 EndBoneCompactIndex;

	/** Percentage of maximum length to extend limb to reach new foot position. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Limb, meta = (PinHiddenByDefault))
	float LimbMaxLengthMultiplier = 0.9f;
	float LimbLength;

	/** Squash / stretch the stride of the feet. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Limb, meta = (PinHiddenByDefault))
	float StepLengthMultiplier = 1.0f;
	float StepLength;

	/** Time in seconds to take a single step from un-planting to planted. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Limb, meta = (UIMin = "0.1", UIMax = "100.0", PinHiddenByDefault))
	float StepDuration = 0.5f;
	float TimeSinceUnPlanted = 0.0f;

	/** Maximum height in centimeters of foot at center of stride.*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Limb, meta = (UIMin = "0.1", UIMax = "100.0", PinHiddenByDefault))
	float StepHeight = 30.0f;

	/** Specify the index of limbs that must be planted before this one can be UN-planted.*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Limb)
	TArray<int32> RelatedLimbs;

	/** How much to orient the foot bone to the ground normal. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Limb, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float RotateFootToGround = 0.9f;

	/** Prevent other feet from colliding with this one.*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Limb, meta = (ClampMin = "0.1", UIMin = "0.1", UIMax = "100.0"))
	float FootCollisionRadius = 10.0f;

	// locomotion state
	bool IsPlanted = true;
	bool WantsToReplant = false;
	FVector StartOrigCompPosition;
	FVector EndOrigCompPosition;
	FQuat EndOrigCompRotation;

	FVector PlantedWorldPosition;
	FQuat PlantedCompRotation;

	FVector CurrentWorldPosition;
	FQuat CurrentCompRotation;

	TArray<FPlaneConstraint> FootPlaneConstraints;

	// ray cast data
	FVector RayDirectionInitial;
	FVector RayStart;
	FVector RayEnd;
	FVector TargetWorldPosition;
	bool RayHit;
	FVector TargetWorldNormal;
	float DistanceToTarget;
};

USTRUCT(BlueprintInternalUseOnly)
struct POWERIKRUNTIME_API FAnimNode_PowerIK_Walk : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Name of joint that acts as the root of the solve. All effectors must be on children of this bone. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Rig, meta = (PinShownByDefault))
	FName CharacterRoot = "pelvis";

	/** How much to rotate root towards neighboring effectors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	float RootRotationMultiplier = 1.0f;

	/** Apply inertial damping to Character Root bone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	FPowerIKBodyInertia BodyInertia;

	/** List of feet that are grounded. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Limbs, meta = (PinHiddenByDefault))
	TArray<FPowerIKWalkingLimb> Limbs;
	FVector LastRootLocationWhenPlanted;

	/** Maximum distance that character root can move from previous planted location before being clamped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Stepping, meta = (ClampMin = "0.1", ClampMax = "100.0", PinHiddenByDefault))
	float StepMaxReachDistance = 200.0f;

	/** Speed up or slow down stepping speed for all limbs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Stepping, meta = (ClampMin = "0.1", ClampMax = "100.0", PinHiddenByDefault))
	float StepDurationMultiplier = 1.0f;

	/** Raise or lower step height for all limbs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Stepping, meta = (ClampMin = "0.1", ClampMax = "100.0", PinHiddenByDefault))
	float StepHeightMultiplier = 1.0f;

	/** Foot height multiplier normalized over the duration of a single step.*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Stepping)
	UCurveFloat* StepHeightCurve;

	/** Foot speed multiplier normalized over the duration of a single step.*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Stepping)
	UCurveFloat* StepSpeedCurve;

	/** Maximum distance in centimeters that feet can get from capsule before character is forcibly teleported. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Stepping)
	float TeleportDistance = 1000.0f;

	/** Print warning to log when limbs are teleported (debugging). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Stepping)
	bool PrintTeleportWarning = false;

	/** The collision channel used for "Grounded" effectors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Collision)
	TEnumAsByte<enum ECollisionChannel> CollisionChannel = ECollisionChannel::ECC_WorldStatic;

	/** Do you want to cast against complex collision geometry (if mesh has it). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Collision)
	bool TraceComplex = true;

	/** List of custom directions for bones to bend in. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKBoneBendDirection> BendDirections;

	/** List of excluded bones. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKExcludedBone> ExcludedBones;

	/** List of rotation limits for joints. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKBoneLimit> JointLimits;

	/** Number of iterations to improve squashing poses. */
	UPROPERTY(EditAnywhere, Category = Solver, meta = (ClampMin = "0", ClampMax = "100", UIMin = "0", UIMax = "100"))
	int32 MaxSquashIterations = 6;

	/** Number of iterations to improve stretching poses. */
	UPROPERTY(EditAnywhere, Category = Solver, meta = (ClampMin = "0", ClampMax = "100", UIMin = "0", UIMax = "100"))
	int32 MaxStretchIterations = 6;

	/** Number of iterations to cleanup final pose. */
	UPROPERTY(EditAnywhere, Category = Solver, meta = (ClampMin = "0", ClampMax = "100", UIMin = "0", UIMax = "100"))
	int32 MaxFinalIterations = 3;

	/** If true, bone lengths will be updated each frame based on the input pose. This allows bone chains to 'stretch' but has a small additional cost to support.*/
	UPROPERTY(EditAnywhere, Category = Solver)
	bool AllowBoneTranslation = false;

	/** Global alpha to blend effector of solver on/off from 0 to 1. */
	UPROPERTY(EditAnywhere, Category = Solver, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"), meta = (PinShownByDefault))
	float SolverAlpha = 1.0f;

	/** Adjust size of debug gizmos. */
	UPROPERTY(EditAnywhere, Category = Debug, meta = (ClampMin = "1", ClampMax = "2000", UIMin = "1", UIMax = "100"))
	float DebugDrawSize = 20.0f;

	UWorld* World;

	FAnimNode_PowerIK_Walk();
	
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
	
	bool InitializeSolverData(const FBoneContainer& RequiredBones);
	void CheckForTeleport(FComponentSpacePoseContext& Output);
	void UpdateLimbTargets(FComponentSpacePoseContext& Output);
	void UpdateLimbsWantToRePlant(FComponentSpacePoseContext& Output);
	void UpdateLimbFeet(FComponentSpacePoseContext& Output);

	void AnimateLimbStep(int32 LimbIndex, const FTransform& WorldToComponent);
	bool AtLeastOneRelatedLimbIsPlanted(const FPowerIKWalkingLimb& Limb);
	bool LimbIsTooFarFromTarget(const FPowerIKWalkingLimb& Limb);
	void EnforceFootPlaneConstraints(int32 LimbIndex);
	void ResolveLimbTargetCollision(int32 LimbIndex);
	float GetLengthOfLimb(const FReferenceSkeleton& RefSkel, FName StartBone, const FName EndBone);
	bool RaySphereIntersection(
		const FVector& LineOrigin,
		const FVector& LineDirection,
		const FVector& SphereCenter,
		float SphereRadius,
		float& OutParam);

	TArray<FPowerIKEffector> FootEffectors;
	FPowerIKCenterOfGravity CenterOfGravityConstraint; // default settings (not used)
	FPowerIKCore Core;
	bool IsInitialized = false;
};



