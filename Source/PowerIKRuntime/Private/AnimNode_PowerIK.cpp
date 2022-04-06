/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

//#pragma optimize( "", off )

#include "AnimNode_PowerIK.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include <string>
#include "PowerIKRuntime/sdk/include/PowerIKSettings.h"

#if WITH_EDITOR
#include "DrawDebugHelpers.h"
#endif

using PowerIK::Vec3;
using PowerIK::Quat;


void FAnimNode_PowerIK::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	World = Context.AnimInstanceProxy->GetSkelMeshComponent()->GetWorld();
}

void FAnimNode_PowerIK::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(CacheBones_AnyThread)
	FAnimNode_Base::CacheBones_AnyThread(Context);
	
	// I have to evaluate graph inputs here in order to get effector bone names
	// from attached pin inputs. Otherwise they are all at default, "None".
	// Normal method is to use 	FAnimNode_SkeletalControlBase::InitializeBoneReferences()
	// but FAnimNode_SkeletalControlBase::CacheBones_AnyThread does not evaluate graph inputs.
	// Likely because none of the default skeletal control nodes use struct inputs as pins.
	GetEvaluateGraphExposedInputs().Execute(Context);

	const FBoneContainer& RequiredBones = Context.AnimInstanceProxy->GetRequiredBones();
	int32 NumBones = RequiredBones.GetCompactPoseNumBones();
	if (Core.CheckSolverNeedsReinitialized(NumBones, Effectors))
	{
		// load skeleton into core
		Core.LoadBonesFromAnimGraph(RequiredBones);
		Core.InitializeSolver(
			CharacterRoot,
			Effectors,
			ExcludedBones,
			BendDirections,
			JointLimits);
	}
	
	ComponentPose.CacheBones(Context);
}

bool FAnimNode_PowerIK::IsValidToEvaluate(
	const USkeleton* Skeleton, 
	const FBoneContainer& RequiredBones)
{
	return Core.IsInitialized; // virtual function called by Unreal
}

void FAnimNode_PowerIK::EvaluateSkeletalControl_AnyThread(
	FComponentSpacePoseContext& Output, 
	TArray<FBoneTransform>& OutBoneTransforms)
{
	SCOPE_CYCLE_COUNTER(STAT_PowerIK_Eval);

	// update effector transforms
	Core.UpdateEffectorBoneTransformsFromAnimGraph(Effectors, Output);
	USkeletalMeshComponent* SkeletalMesh = Output.AnimInstanceProxy->GetSkelMeshComponent();
	FTransform WorldToComponent = SkeletalMesh->GetComponentToWorld().Inverse();
	Core.UpdateEffectorTransforms(Effectors, WorldToComponent);

	// copy input pose/settings to solver
	Core.SetSolverBoneTransformsFromAnimGraph(Output);
	Core.SetSolverInputs(
		RootRotationMultiplier, 
		MaxSquashIterations, 
		MaxStretchIterations,
		MaxFinalIterations,
		AllowBoneTranslation,
		SmoothingMaxSpeedMultiplier, 
		SmoothingMaxDistanceMultiplier, 
		CenterOfGravityConstraint,
		BodyInertia,
		Effectors);

	// effector positions rarely make sense in editor (need runtime input)
	// so avoid doing IK solve in editor views
	if (World->WorldType == EWorldType::EditorPreview ||
		World->WorldType == EWorldType::Editor)
	{
		return;
	}

	// run the solver
	{	
		SCOPE_CYCLE_COUNTER(STAT_PowerIK_Solve);
		Core.Solver->Solve(World->DeltaTimeSeconds, SolverAlpha);
	}

	// copy the solver results
	Core.CopySolverOutputToAnimGraph(OutBoneTransforms, Output);
}

void FAnimNode_PowerIK::GatherDebugData(FNodeDebugData& DebugData)
{
	// called from command: "showdebug animation"

	// draw effector locations
	Core.DebugDrawEffectorsInEditor(DebugData, Effectors, DebugDrawSize);

	// gather on-screen debug text
	FString DebugLine = DebugData.GetNodeName(this);
	DebugLine += "(";
	AddDebugNodeData(DebugLine);
	DebugLine += FString::Printf(TEXT(" Num Effectors: %d)"), Effectors.Num());
	DebugData.AddDebugItem(DebugLine);
	ComponentPose.GatherDebugData(DebugData);
}

#if WITH_EDITOR
void FAnimNode_PowerIK::ConditionalDebugDraw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* MeshComp) const
{
	FTransform LocalToWorld = MeshComp->GetComponentToWorld();
	Core.DebugDrawEffectorsInPreview(PDI, DebugDrawSize, LocalToWorld, Effectors);
	Core.DebugDrawJointLimits(PDI, DebugDrawSize);
}
#endif // WITH_EDITOR
