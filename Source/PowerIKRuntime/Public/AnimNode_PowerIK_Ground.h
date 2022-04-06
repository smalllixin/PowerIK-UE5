/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include <string>
#include "CoreMinimal.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"

#include "PowerIKRuntime/sdk/include/PowerIKGround.h"
#include "PowerIKRuntime/Public/PowerIK_UnrealCore.h"

#include "AnimNode_PowerIK_Ground.generated.h"

DECLARE_CYCLE_STAT(TEXT("PowerIK Ground Total"), STAT_PowerIK_Ground_Eval, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("PowerIK Ground Solve"), STAT_PowerIK_Ground_Solve, STATGROUP_Anim);

USTRUCT(BlueprintInternalUseOnly)
struct POWERIKRUNTIME_API FPowerIKGroundFoot
{
	GENERATED_USTRUCT_BODY()

	/** Name of foot joint. This will be tip of IK effect. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Foot)
	FName BoneName;

	/** How much this effector pulls un-affected parts of body. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"))
	float PullWeight = 1.0f;

	/** Use normalized PullWeight values in solver. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	bool NormalizePulling = true;

	/** Positive direction scale factor for effector weights. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	FVector PositivePullFactor = FVector(0.0f, 0.0f, 1.0f);

	/** Negative direction scale factor for effector weights. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	FVector NegativePullFactor = FVector(0.0f, 0.0f, 1.0f);

	// wall collision
	int LimbRootBoneIndex = -1;

	// ray cast debug data
	FVector RayStart;
	FVector RayEnd;
	FVector RayHitPosition;
	bool RayHit;
	FVector GroundNormal;
};

USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKGroundCollision
{
	GENERATED_BODY()

public:

	/** The collision channel used for "Grounded" effectors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings)
	TEnumAsByte<enum ECollisionChannel> CollisionChannel = ECollisionChannel::ECC_WorldStatic;

	/** Do you want to cast against complex collision geometry (if mesh has it). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings)
	bool TraceComplex = true;

	/** Maximum height to move Grounded effector UPWARDS. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float RayCastUp = 200.0f;

	/** Maximum height to move Grounded effector DOWNWARDS. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float RayCastDown = 200.0f;

	/** Will raycast sideways to prevent feet from penetrating walls. Requires twice as many raycasts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings)
	bool EnableWallCollision = false;

	/** Distance to push feet away from walls when they collide with a wall. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float WallOffset = 20.0f;
};


USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKGroundSlope
{
	GENERATED_BODY()

public:

	/** Normalized vector that describes the direction of travel. */
	UPROPERTY(EditAnywhere, Category = Ground, BlueprintReadWrite)
	FVector StrideDirection = FVector(0.0f, 1.0f, 0.0f);
	/** Maximum angle relative to gravity vector to apply leaning and stride scaling. */
	UPROPERTY(EditAnywhere, Category = GroundAngle, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "90", UIMin = "0", UIMax = "90"))
	float MaxGroundAngle = 45.0f;
	/** Maximum speed, in degrees/second, that body angle will adjust to changing slopes. */
	UPROPERTY(EditAnywhere, Category = GroundAngle, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "180", UIMin = "0", UIMax = "180"))
	float MaxNormalAngularSpeed = 40.0f;
	
	/** If true, entire skeleton is rotated to align to the ground. Useful for quadrupeds.*/
	UPROPERTY(EditAnywhere, Category = Strude, BlueprintReadWrite)
	bool OrientToGround = false;
	/** Orient entire skeleton to the PITCH direction of ground angle. Range is -1 to 1. Default 0.9. */
	UPROPERTY(EditAnywhere, Category = GroundAngle, BlueprintReadWrite, meta = (ClampMin = "-1", ClampMax = "1", UIMin = "-1", UIMax = "1"))
	float OrientToPitch = 0.8f;
	/** Orient entire skeleton to the ROLL direction of ground angle. Range is -1 to 1. Default 0.1. */
	UPROPERTY(EditAnywhere, Category = GroundAngle, BlueprintReadWrite, meta = (ClampMin = "-1", ClampMax = "1", UIMin = "-1", UIMax = "1"))
	float OrientToRoll = 0.1f;

	/** If true, stride of IsGrounded effectors will be scaled in Stride Direction. */
	UPROPERTY(EditAnywhere, Category = Strude, BlueprintReadWrite)
	bool ScaleStride = false;
	/** Percentage to scale stride when going uphill. Range is 0-1. Default 0.4.*/
	UPROPERTY(EditAnywhere, Category = Stride, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float UphillStrideScale = 0.4f;
	/** Percentage to scale stride when going downhill. Range is 0-1. Default 0.2.*/
	UPROPERTY(EditAnywhere, Category = Stride, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float DownhillStrideScale = 0.2f;
	/** Percentage to scale stride when on a side-hill. Range is 0-1. Default 1.0.*/
	UPROPERTY(EditAnywhere, Category = Stride, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float SidehillStrideScale = 1.0f;
	/** Percentage (in centimeters) to push feet outwards on sidehills. Default is 20cm.*/
	UPROPERTY(EditAnywhere, Category = Stride, BlueprintReadWrite, meta = (UIMin = "0", UIMax = "1000"))
	float SidehillPushOuterFeet = 20.0f;

	/** If True, character root bone is rotated in opposite direction of ground normal. */
	UPROPERTY(EditAnywhere, Category = Lean, BlueprintReadWrite)
	bool Lean = false;
	/** Amount to lean forward when going uphill. Range is 0-1. Default 0.3.*/
	UPROPERTY(EditAnywhere, Category = Lean, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float UphillLean = 0.3f;
	/** Amount to lean backward when going downhill. Range is 0-1. Default 0.5.*/
	UPROPERTY(EditAnywhere, Category = Lean, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float DownhillLean = 0.5f;
	/** Amount to lean sideways into side-hills. Range is 0-1. Default 0.0.*/
	UPROPERTY(EditAnywhere, Category = Lean, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float SidehillLean = 0.0f;

	/** If True, will counter lean CounterLeanBone. Usually this is the head. */
	UPROPERTY(EditAnywhere, Category = CounterLean, BlueprintReadWrite)
	bool CounterLean = false;
	/** Name of base of counter lean bone. Usually this is the base Head bone.*/
	UPROPERTY(EditDefaultsOnly, Category = LeCounterLeanan, BlueprintReadWrite)
	FName CounterLeanBoneName = "head";
	int32 CounterLeanBoneRefSkelIndex;
	/** Amount to counter lean backward when going uphill. Range is 0-1. Default 0.8.*/
	UPROPERTY(EditAnywhere, Category = CounterLean, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float UphillCounterLean = 0.8f;
	/** Amount to counter lean backward when going downhill. Range is 0-1. Default 1.0.*/
	UPROPERTY(EditAnywhere, Category = CounterLean, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float DownhillCounterLean = 1.0f;
	/** Amount to counter lean sideways into side-hills. Range is 0-1. Default 0.0.*/
	UPROPERTY(EditAnywhere, Category = CounterLean, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float SidehillCounterLean = 0.0f;

	/** If True, character root bone position is gradually pushed vertically and horizontally. */
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite)
	bool MoveRoot = false;
	/** Amount (in centimeters) to push the character root up/down when going UP hill. Default is 10.0.*/
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite, meta = (UIMin = "-100", UIMax = "100"))
	float UphillVertOffset = 10.0f;
	/** Amount (in centimeters) to push the character root forward/back when going UP hill. Default is -20.0.*/
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite, meta = (UIMin = "-100", UIMax = "100"))
	float UphillHorizOffset = -20.0f;
	/** Amount (in centimeters) to push the character root up/down when going DOWN hill. Default is 10.0.*/
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite, meta = (UIMin = "-100", UIMax = "100"))
	float DownhillVertOffset = 10.0f;
	/** Amount (in centimeters) to push the character root forward/back when going DOWN hill. Default is 20.0.*/
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite, meta = (UIMin = "-100", UIMax = "100"))
	float DownhillHorizOffset = 20.0f;
	/** Amount (in centimeters) to push the character root forward/back when on a SIDE hill. Default is -10.0.*/
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite, meta = (UIMin = "-100", UIMax = "100"))
	float SidehillHorizOffset = -10.0f;
	/** Amount (in centimeters) to push the character root up/down when on a SIDE hill. Default is -10.0.*/
	UPROPERTY(EditAnywhere, Category = MoveRoot, BlueprintReadWrite, meta = (UIMin = "-100", UIMax = "100"))
	float SidehillVertOffset = -10.0f;

	/** If True, feet are oriented to ground normal. */
	UPROPERTY(EditAnywhere, Category = FootRotation, BlueprintReadWrite)
	bool RotateFootToGround = false;
	/** Amount to pitch foot to orient to up/down hills. Range is 0-1. Default 1.0.*/
	UPROPERTY(EditAnywhere, Category = FootRotation, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float PitchFootAmount = 1.0f;
	/** Amount to roll foot to orient to sidehills. Range is 0-1. Default 0.6.*/
	UPROPERTY(EditAnywhere, Category = FootRotation, BlueprintReadWrite, meta = (ClampMin = "-10", ClampMax = "10", UIMin = "-1", UIMax = "1"))
	float RollFootAmount = 0.6f;
	/** Speed in degrees per second that feet adjust to changing ground normals. Instant if <= 0.0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Rig, meta = (ClampMin = "0", ClampMax = "10000", UIMin = "0", UIMax = "1000", PinHiddenByDefault))
	float FootAngleDeltaSmoothSpeed = 0.0f;

	/** If True, applies static offset to foot positions along normal direction. Useful for nudging feet into contact with ground. */
	UPROPERTY(EditAnywhere, Category = FootOffset, BlueprintReadWrite)
	bool OffsetFeetPositions = false;
	/** Amount in centimeters to offset the foot in the direction of the foot's ground normal. */
	UPROPERTY(EditAnywhere, Category = FootOffset, BlueprintReadWrite, meta = (ClampMin = "-1000", ClampMax = "1000", UIMin = "-10", UIMax = "10"))
	float StaticFootOffset = 4.0f;
};

USTRUCT(BlueprintInternalUseOnly)
struct POWERIKRUNTIME_API FAnimNode_PowerIK_Ground : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Name of joint that acts as the root of the solve. All effectors must be on children of this bone. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Rig, meta = (PinShownByDefault))
	FName CharacterRoot = "pelvis";

	/** Apply inertial damping to character body. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	FPowerIKBodyInertia BodyInertia;

	/** List of feet that are grounded. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	TArray<FPowerIKGroundFoot> Feet;

	/** Speed in centimeters per second that feet adjust to changing positions. Ignored if <= 0.0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Rig, meta = (ClampMin = "0", ClampMax = "10000", UIMin = "0", UIMax = "1000", PinHiddenByDefault))
	float FeetDeltaSmoothSpeed = 200.0f;

	/** How much to rotate root towards neighboring effectors. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Rig, meta = (PinHiddenByDefault))
	float RootRotationMultiplier = 1.0f;

	/** Settings for ray-casting to place feet on ground. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = GroundSettings)
	FPowerIKGroundCollision GroundCollision;

	/** Settings for leaning on slopes and scaling stride lengths. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GroundSettings, meta = (PinHiddenByDefault))
	FPowerIKGroundSlope GroundSlope;

	/** List of custom directions for bones to bend in. */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKBoneBendDirection> BendDirections;

	/** List of excluded bones. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKExcludedBone> ExcludedBones;

	/** List of rotation limits for joints. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	TArray<FPowerIKBoneLimit> JointLimits;

	/** Center of Gravity Constraint, applied to Character Root bone. */
	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Constraints)
	FPowerIKCenterOfGravity CenterOfGravityConstraint;

	/** Number of iterations to improve squashing poses. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "0", ClampMax = "100", UIMin = "0", UIMax = "100"))
	int32 MaxSquashIterations = 6;

	/** Number of iterations to improve stretching poses. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "0", ClampMax = "100", UIMin = "0", UIMax = "100"))
	int32 MaxStretchIterations = 6;

	/** Number of iterations to cleanup final pose. */
	UPROPERTY(EditAnywhere, Category = SolverSettings, meta = (ClampMin = "0", ClampMax = "100", UIMin = "0", UIMax = "100"))
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
	void PutEffectorsOnGround(FComponentSpacePoseContext& Output);
	void AlignToSlope(FComponentSpacePoseContext& Output);
	void RayCastEffectorToGround(
		FComponentSpacePoseContext& Output,
		USkeletalMeshComponent* SkeletalMesh,
		int32 EffectorIndex);
	void RayCastEffectorToWall(
		FComponentSpacePoseContext& Output,
		USkeletalMeshComponent* SkeletalMesh,
		int32 EffectorIndex);

	TArray<FPowerIKEffector> FootEffectors;
	std::vector<PowerIK::GroundFoot> GroundFeet;
	PowerIK::GroundAlign Ground;
	FPowerIKCore Core;
	bool IsInitialized = false;
};
