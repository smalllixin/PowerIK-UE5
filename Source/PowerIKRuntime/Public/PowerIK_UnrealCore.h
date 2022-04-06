/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include <string>
#include "CoreMinimal.h"
#include "Containers/Array.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"

#include "PowerIKRuntime/sdk/include/PowerIK.h"
#include "PowerIKRuntime/sdk/include/PowerIKMath.h"
#include "Rigs/RigHierarchy.h"

#include "PowerIK_UnrealCore.generated.h"

// conversion helper for PowerIK's internal data types
PowerIK::Vec3 FVectorToVec3(FVector V);
FVector Vec3ToFVector(PowerIK::Vec3 V);
PowerIK::Quat FQuatToQuat(FQuat Q);
FQuat QuatToFQuat(PowerIK::Quat Q);


USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKBoneLimit
{
	GENERATED_BODY()

	UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite, meta = (Input, Constant, CustomWidget = "BoneName"))
	FName BoneName = "None";

	UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite)
	float Stiffness = 0.0f;

	// WIP joint limits are experimental feature under construction.
	//UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite)
	bool UseLimits = false;
	//UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite)
	FVector MinAngles = FVector(-20.0f, -20.0f, -20.0f);
	//UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite)
	FVector MaxAngles = FVector(20.0f, 20.0f, 20.0f);
	//UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite)
	bool NegateXAxis = false;
};


USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKBoneBendDirection
{
	GENERATED_BODY()

	UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite, meta = (Input, Constant, CustomWidget = "BoneName"))
	FName BoneName = "None";

	UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite)
	FVector BendDirection = FVector(0.0f, 1.0f, 0.0f);
};

USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKExcludedBone
{
	GENERATED_BODY()

	UPROPERTY(EditDefaultsOnly, Category = Joint, BlueprintReadWrite, meta = (Input, Constant, CustomWidget = "BoneName"))
	FName BoneName = "None";
};

UENUM(BlueprintType)
enum EPoleVectorModeEnum
{
	PV_None 		UMETA(DisplayName = "None"),
	PV_Position 	UMETA(DisplayName = "Character Space Position"),
	PV_Bone 		UMETA(DisplayName = "Bone"),
	PV_AngleOffset	UMETA(DisplayName = "Float Angle Offset"),
};


USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKPoleVector
{
	GENERATED_BODY()

	/** Type of Pole Vector. */
	UPROPERTY(EditAnywhere, Category = PoleVector, BlueprintReadWrite)
	TEnumAsByte<EPoleVectorModeEnum> Mode = EPoleVectorModeEnum::PV_None;

	UPROPERTY(EditDefaultsOnly, Category = PoleVector, BlueprintReadWrite)
	FVector Position = FVector(0.0f, 100.0f, 0.0f);
	
	UPROPERTY(EditDefaultsOnly, Category = PoleVector, BlueprintReadWrite, meta = (Input, Constant, CustomWidget = "BoneName"))
	FName BoneName = "None";
	unsigned int BoneIndex = 0;

	UPROPERTY(EditDefaultsOnly, Category = PoleVector, BlueprintReadWrite)
	float AngleOffset = 0.0f;
};


USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKSmoothing
{
	GENERATED_BODY()

	/** Apply temporal smoothing to effector location. */
	UPROPERTY(EditAnywhere, Category = Ground, BlueprintReadWrite)
	bool SmoothPositionOverTime = false;

	/** Maximum speed effector can react to input position when smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Grounding, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float MaxPositionSpeed = 200.0f;

	/** Maximum distance effector can be from input position when smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Grounding, meta = (ClampMin = "0.1", ClampMax = "2000", UIMin = "0.1", UIMax = "2000"))
	float MaxPositionDistance = 1000.0f;

	/** Apply temporal smoothing to effector location. */
	UPROPERTY(EditAnywhere, Category = Ground, BlueprintReadWrite)
	bool SmoothRotationOverTime = false;

	/** Maximum speed, in degrees/second, effector can react to input rotation when smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Grounding, meta = (ClampMin = "0.1", ClampMax = "360", UIMin = "0.1", UIMax = "360"))
	float MaxDegreesSpeed = 90.0f;

	/** Maximum distance, in degrees, effector can be from input rotation when smoothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Grounding, meta = (ClampMin = "0.1", ClampMax = "360", UIMin = "0.1", UIMax = "360"))
	float MaxDegreesDistance = 135.0f;
};

USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKCenterOfGravity
{
	GENERATED_BODY()

	/** Smoothly blend effect of this constraint on/off. */
	UPROPERTY(EditAnywhere, Category = Constraint, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float Alpha = 0.0f;

	/** How much to pull character root towards feet in horiz direction. */
	UPROPERTY(EditAnywhere, Category = Constraint, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float HorizAmount = 1.2f;

	/** How much to pull character root downwards when root pulled away from feet. */
	UPROPERTY(EditAnywhere, Category = Constraint, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float VertAmount = 0.3f;

	/** How much to pull the root with the rest of the body. */
	UPROPERTY(EditAnywhere, Category = Constraint, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10", UIMin = "0", UIMax = "1"))
	float PullBodyAmount = 0.4f;
};

USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKBodyInertia
{
	GENERATED_BODY()

	/** Whether to use inertia to smooth root motion. Default is false.*/
	UPROPERTY(EditDefaultsOnly, Category = Inertia, BlueprintReadWrite, meta = (Input, Constant))
	bool ApplyInertiaToBody = false;

	/** The strength of the spring attached the root to the solved position. */
	UPROPERTY(EditAnywhere, Category = Inertia, BlueprintReadWrite, meta = (ClampMin = "1", ClampMax = "1000", UIMin = "0", UIMax = "100", Input, EditCondition = "!UseSpring"))
	float SmoothFactor = 5.0f;

	/** Use a spring instead of smooth motion (can overshoot). Default is false.*/
	UPROPERTY(EditDefaultsOnly, Category = Inertia, BlueprintReadWrite, meta = (Input, Constant))
	bool UseSpring = false;

	/** The strength of the spring attached the root to the solved position. */
	UPROPERTY(EditAnywhere, Category = Inertia, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "1000", UIMin = "0", UIMax = "100", Input, EditCondition = "UseSpring"))
	float SpringStrength = 100.0f;

	/** Dampen the motion over time. Range is 0-1. Default is 0.2. )*/
	UPROPERTY(EditAnywhere, Category = Inertia, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1", Input, EditCondition = "UseSpring"))
	float SpringDamping = 0.2f;
};


UENUM(BlueprintType)
enum EEffectorSpaceEnum
{
	ES_Additive 	UMETA(DisplayName = "Relative to Input Pose"),
	ES_World 		UMETA(DisplayName = "World Space"),
	ES_Component	UMETA(DisplayName = "Component Space")
};

USTRUCT(BlueprintType)
struct POWERIKRUNTIME_API FPowerIKEffector
{
	GENERATED_BODY()

	/** The bone to affect. */
	UPROPERTY(EditDefaultsOnly, Category = Effector, BlueprintReadWrite, meta = (Input, Constant, CustomWidget = "BoneName"))
	FName BoneName = "";

	/** Where to move this effector. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	FVector Position = FVector::ZeroVector;

	/** The space to consider the position in. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	TEnumAsByte<EEffectorSpaceEnum> PositionSpace = EEffectorSpaceEnum::ES_Additive;

	/** Rotation used by RotateBone and RotateLimb. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	FRotator Rotation = FRotator::ZeroRotator;

	/** The space to consider the rotation in. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	TEnumAsByte<EEffectorSpaceEnum> RotationSpace = EEffectorSpaceEnum::ES_Additive;

	/** How much this effector pulls un-affected parts of body. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"))
	float PullWeight = 1.0f;

	/** Use normalized PullWeight values in solver. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	bool NormalizePulling = true;

	/** Positive direction scale factor for effector weights. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	FVector PositivePullFactor = FVector::OneVector;

	/** Negative direction scale factor for effector weights. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	FVector NegativePullFactor = FVector::OneVector;

	/** Does this effector rotate the bone it affects? */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	bool RotateBone = false;

	/** Does this effector rotate the limb surrounding it? */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite)
	bool RotateLimb = false;

	/** Speed in centimeters per second that this effector adjusts to changing positions. Ignored if <= 0.0. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10000", UIMin = "0", UIMax = "1000"))
	float DeltaSmoothSpeed = 0.0f;

	/** Speed in degrees per second that this effector adjusts to changing rotations. Ignored if <= 0.0. */
	UPROPERTY(EditAnywhere, Category = Effector, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "10000", UIMin = "0", UIMax = "1000"))
	float AngularDeltaSmoothSpeed = 0.0f;

	/** Analytic, simple velocity clamping on effector position, rotation and distance to input.*/
	UPROPERTY(EditAnywhere, Category = Smoothing, BlueprintReadWrite)
	FPowerIKSmoothing Smoothing;

	/** Optional, explicit control over the direction of the limb controlled by this effector.*/
	UPROPERTY(EditAnywhere, Category = PoleVector, BlueprintReadWrite)
	FPowerIKPoleVector PoleVector;

	/** Does this effector pull the center of gravity? (usually only feet). */
	UPROPERTY(EditAnywhere, Category = COG, BlueprintReadWrite)
	bool AffectsCenterOfGravity = false;

	/** Blend entire effector on/off */
	UPROPERTY(EditAnywhere, Category = Blend, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "1", UIMin = "0", UIMax = "1"))
	float Alpha = 1.0f;

	// store animated positions, used to:
	// - make inputs relative to skeleton
	// - allow debug viewing of unaffected transforms
	FVector BoneCompPosition = FVector::ZeroVector;
	FQuat BoneCompRotation = FQuat::Identity;

	// final transform sent to solver
	FVector OutPosition;
	FQuat OutRotation;
};

USTRUCT()
struct POWERIKRUNTIME_API FPowerIKEffectorData
{
	GENERATED_BODY()
	
	std::string NameString = "";			// string version of the name
	int32 RefSkelBoneIndex = INDEX_NONE;	// the index of bone in reference skeleton
	int32 CompactBoneIndex = INDEX_NONE;	// the index of the bone in the compact pose
	int32 SolverEffectorIndex = INDEX_NONE;	// the index of this effector in the solver
};

USTRUCT()
struct POWERIKRUNTIME_API FPowerIKBoneData
{
	GENERATED_BODY()
	
	FName BoneName = "";			// name of the bone
	std::string NameString = "";
	
	int32 Index = INDEX_NONE;		// index of the bone in the reference skeleton
	int32 ParentIndex = INDEX_NONE;	// index of parent bone in the compact pose

	bool IsExcluded = false;		// is this bone excluded from the solve?

	float Stiffness = 0.0f;
	bool UseLimits = false;
	FVector MinLimits = FVector::ZeroVector;
	FVector MaxLimits = FVector::ZeroVector;
	bool NegateXAxis = false;

	bool UseCustomBendDirection = false;
	FVector BendDirection = FVector::UpVector;
};

USTRUCT()
struct POWERIKRUNTIME_API FPowerIKRootData
{
	GENERATED_BODY()
	
	std::string BoneName;
	int32 RefSkelIndex;
};

struct FRigBoneHierarchy;

USTRUCT()
struct POWERIKRUNTIME_API FPowerIKCore
{
	GENERATED_BODY()
	
	FPowerIKCore(){};
	~FPowerIKCore();
	FPowerIKCore(const FPowerIKCore&);
	void operator=(const FPowerIKCore&);

	void Reset();

	bool CheckSolverNeedsReinitialized(
		int32 NumBonesInSkeleton,
		TArray<FPowerIKEffector>& Effectors) const;

	void LoadBonesFromAnimGraph(const FBoneContainer& RequiredBones);

	void LoadBonesFromControlRig(URigHierarchy* Hierarchy);

	int32 GetBoneIndex(FName BoneName);
	int32 GetCompactIndex(int32 RefSkeletonIndex);
	FTransform GetRefPoseOfBoneInComponentSpace(
		const FReferenceSkeleton& RefSkel,
		const FName& BoneName);
	
	bool InitializeSolver(
		const FName CharacterRoot,
		const TArrayView<FPowerIKEffector>& Effectors, // bone indices filled out here, only need names set
		const TArrayView<FPowerIKExcludedBone>& ExcludedBones,
		const TArrayView<FPowerIKBoneBendDirection>& BendDirections,
		const TArrayView<FPowerIKBoneLimit>& JointLimits);

	void UpdateEffectorBoneTransformsFromAnimGraph(
		const TArrayView<FPowerIKEffector>& Effectors, 
		FComponentSpacePoseContext& Output);

	void UpdateEffectorBoneTransformsFromControlRig(
		const TArrayView<FPowerIKEffector>& Effectors,
		URigHierarchy* Hierarchy);

	void UpdateEffectorTransforms(
		const TArrayView<FPowerIKEffector>& Effectors, 
		FTransform WorldToComponent);

	void SetSolverBoneTransformsFromAnimGraph(FComponentSpacePoseContext& Output);
	
	void SetSolverBoneTransformsFromControlRig(URigHierarchy* Hierarchy);
	
	void SetSolverInputs(
		float RootRotationMultiplier,
		int32 MaxSquashIterations,
		int32 MaxStretchIterations,
		int32 MaxFinalIterations,
		bool AllowBoneTranslation,
		float SmoothingMaxSpeedMultiplier,
		float SmoothingMaxDistanceMultiplier,
		const FPowerIKCenterOfGravity& CenterOfGravity,
		const FPowerIKBodyInertia& RootInertia,
		const TArrayView<FPowerIKEffector>& Effectors);

	void CopySolverOutputToAnimGraph(
		TArray<FBoneTransform>& OutBoneTransforms,
		FComponentSpacePoseContext& Output);

	void CopySolverOutputToControlRig(URigHierarchy* Hierarchy);

	void DebugDrawEffectorsInPreview(
		FPrimitiveDrawInterface* PDI, 
		float DebugDrawSize, 
		const FTransform& LocalToWorld,
		const TArray<FPowerIKEffector>& Effectors) const;

	void DebugDrawEffectorsInEditor(
		FNodeDebugData& DebugData, 
		const TArray<FPowerIKEffector>& Effectors, 
		float DebugDrawSize) const;
	
	void DebugDrawJointLimits(
		FPrimitiveDrawInterface* PDI, 
		float DebugDrawSize) const;

	FPowerIKRootData Root;
	TArray<FPowerIKBoneData> BonesData;
	TMap<FName, int32> BoneNameToIndexMap;
	TArray<FPowerIKEffectorData> EffectorsData;
	PowerIK::Solver* Solver = nullptr;
	bool IsInitialized = false;
};