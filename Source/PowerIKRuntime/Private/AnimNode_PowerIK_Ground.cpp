/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

//#pragma optimize( "", off )

#include "AnimNode_PowerIK_Ground.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include <string>
#include "PowerIKRuntime/sdk/include/PowerIKSettings.h"
#include "PowerIKRuntime/sdk/include/PowerIKGround.h"

#if WITH_EDITOR
#include "DrawDebugHelpers.h"
#endif

using PowerIK::Vec3;
using PowerIK::Quat;


void FAnimNode_PowerIK_Ground::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	World = Context.AnimInstanceProxy->GetSkelMeshComponent()->GetWorld();
}

void FAnimNode_PowerIK_Ground::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
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
	if (!IsInitialized || Core.CheckSolverNeedsReinitialized(NumBones, FootEffectors))
	{
		IsInitialized = InitializeSolverData(RequiredBones);
	}
	
	ComponentPose.CacheBones(Context);
}

bool FAnimNode_PowerIK_Ground::IsValidToEvaluate(
	const USkeleton* Skeleton, 
	const FBoneContainer& RequiredBones)
{
	return IsInitialized; // virtual function called by Unreal
}

void FAnimNode_PowerIK_Ground::EvaluateSkeletalControl_AnyThread(
	FComponentSpacePoseContext& Output, 
	TArray<FBoneTransform>& OutBoneTransforms)
{
	SCOPE_CYCLE_COUNTER(STAT_PowerIK_Ground_Eval);

	// raycast all foot effector to the ground
	PutEffectorsOnGround(Output);

	// apply slope adjustments to skeleton and effectors
	AlignToSlope(Output);

	// copy input pose/settings to solver
	Core.SetSolverBoneTransformsFromAnimGraph(Output);
	Core.SetSolverInputs(
		RootRotationMultiplier,
		MaxSquashIterations,
		MaxStretchIterations,
		MaxFinalIterations,
		AllowBoneTranslation,
		1.0f, // max smooth speed multiplier (constant)
		1.0f, // max smooth distance multiplier (constant)
		CenterOfGravityConstraint,
		BodyInertia,
		FootEffectors);

	// effector positions rarely make sense in editor (need runtime input)
	// so avoid doing IK solve in editor views
	if (World->WorldType == EWorldType::EditorPreview ||
		World->WorldType == EWorldType::Editor)
	{
		return;
	}

	// run the solver
	{	
		SCOPE_CYCLE_COUNTER(STAT_PowerIK_Ground_Solve);
		Core.Solver->Solve(World->DeltaTimeSeconds, SolverAlpha);
	}

	// copy the results
	Core.CopySolverOutputToAnimGraph(OutBoneTransforms, Output);
}

void FAnimNode_PowerIK_Ground::PutEffectorsOnGround(FComponentSpacePoseContext& Output)
{
	USkeletalMeshComponent* SkeletalMesh = Output.AnimInstanceProxy->GetSkelMeshComponent();

	// move foot effectors to grounded location
	for (int i = 0; i < FootEffectors.Num(); ++i)
	{
		FPowerIKEffectorData& EftrData = Core.EffectorsData[i];
		if (EftrData.SolverEffectorIndex == INDEX_NONE)
		{
			continue; // should never happen
		}
		
		FPowerIKEffector& Eftr = FootEffectors[i];
		FPowerIKGroundFoot& Foot = Feet[i];
		
		// move effector to bone transform in component space
		FTransform EftrBoneCompTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(EftrData.CompactBoneIndex));
		Eftr.OutPosition = EftrBoneCompTransform.GetLocation();
		Eftr.OutRotation = EftrBoneCompTransform.GetRotation();
		// store animated position for debug drawing
		Eftr.BoneCompPosition = Eftr.OutPosition;

		// update effector settings
		Eftr.DeltaSmoothSpeed = FeetDeltaSmoothSpeed;
		Eftr.AngularDeltaSmoothSpeed = GroundSlope.FootAngleDeltaSmoothSpeed;
		Eftr.PullWeight = Foot.PullWeight;
		Eftr.NormalizePulling = Foot.NormalizePulling;
		Eftr.PositivePullFactor = Foot.PositivePullFactor;
		Eftr.NegativePullFactor = Foot.NegativePullFactor;

		// do raycasting to move effectors from foot location to ground (vertically)
		RayCastEffectorToGround(Output, SkeletalMesh, i);

		// optionally raycast each limb to push feet outside of walls
		if (GroundCollision.EnableWallCollision)
		{
			RayCastEffectorToWall(Output, SkeletalMesh, i);
		}

		// copy foot effector locations to ground align solver data
		PowerIK::GroundFoot& GroundFoot = GroundFeet[i];
		GroundFoot.Transform.Position = FVectorToVec3(Eftr.OutPosition);
		GroundFoot.Transform.Rotation = FQuatToQuat(Eftr.OutRotation);
		GroundFoot.Normal = FVectorToVec3(Foot.GroundNormal);
	}
}

void FAnimNode_PowerIK_Ground::RayCastEffectorToGround(
	FComponentSpacePoseContext& Output,
	USkeletalMeshComponent* SkeletalMesh,
	int32 EffectorIndex)
{
	// don't raycast in editor view
	if (World->WorldType == EWorldType::EditorPreview ||
		World->WorldType == EWorldType::Editor)
	{
		return;
	}

	// get foot and it's effector
	FPowerIKEffector& Eftr = FootEffectors[EffectorIndex];
	FPowerIKEffectorData& EftrData = Core.EffectorsData[EffectorIndex];
	FPowerIKGroundFoot& Foot = Feet[EffectorIndex];

	// get component-to-world transform
	FVector Up = SkeletalMesh->GetUpVector();
	// use up as default ground normal until a raycast hits
	Foot.GroundNormal = Up;

	// transform from character to world space
	FTransform ComponentToWorld = SkeletalMesh->GetComponentToWorld();

	// get location of foot in world space
	FVector AnimatedWorldPosition = ComponentToWorld.TransformPosition(Eftr.OutPosition);

	// ray-cast from top to bottom
	FVector RayUpOffset = Up * GroundCollision.RayCastUp;
	FVector RayDownOffset = Up * GroundCollision.RayCastDown;
	Foot.RayStart = AnimatedWorldPosition + RayUpOffset;
	Foot.RayEnd = AnimatedWorldPosition - RayDownOffset;
	Foot.RayHit = false;
	FCollisionQueryParams CollisionParams(FName("PowerIK"), GroundCollision.TraceComplex);
	FHitResult OutHit;

	// run ray-cast
	if (World->LineTraceSingleByChannel(
		OutHit,
		Foot.RayStart,
		Foot.RayEnd,
		GroundCollision.CollisionChannel,
		CollisionParams))
	{
		if (OutHit.bBlockingHit)
		{
			// record that ray hit and it's location
			Foot.RayHit = true;
			Foot.RayHitPosition = OutHit.ImpactPoint;
			
			// store the ground normal under this foot
			// convert world space normal into component space
			FTransform WorldToComponent = ComponentToWorld.Inverse();
			Foot.GroundNormal = WorldToComponent.TransformVector(OutHit.Normal);
			// store the component space position of this foot
			FVector CompSpaceImpact = WorldToComponent.TransformPosition(OutHit.ImpactPoint);
			// add the animated position as an offset
			FVector AnimatedOffset = Eftr.OutPosition.Z * FVector::UpVector;
			Eftr.OutPosition = CompSpaceImpact + AnimatedOffset;
		}
	}
}

void FAnimNode_PowerIK_Ground::RayCastEffectorToWall(
	FComponentSpacePoseContext& Output, 
	USkeletalMeshComponent* SkeletalMesh, 
	int32 EffectorIndex)
{
	// don't raycast in editor view
	if (World->WorldType == EWorldType::EditorPreview ||
		World->WorldType == EWorldType::Editor)
	{
		return;
	}

	// cannot query limb root until solver is initialized (after first tick)
	if (!Core.Solver->IsInitialized())
	{
		return;
	}

	// get root of limb
	FPowerIKGroundFoot& Foot = Feet[EffectorIndex];
	if (Foot.LimbRootBoneIndex == -1)
	{
		FPowerIKEffectorData& EftrData = Core.EffectorsData[EffectorIndex];
		PowerIK::PowerIKErrorCode Result = Core.Solver->GetLimbRoot(EftrData.CompactBoneIndex, Foot.LimbRootBoneIndex);
		if (Result != PowerIK::PowerIKErrorCode::SUCCESS)
		{
			return;
		}
	}

	// START OF RAYCAST - get root of limb
	FTransform LimbRootBoneCompTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(Foot.LimbRootBoneIndex));
	// END OF RAYCAST - get effector (already has position set to bone position)
	FPowerIKEffector& Eftr = FootEffectors[EffectorIndex];

	// transform from character to world space
	FTransform ComponentToWorld = SkeletalMesh->GetComponentToWorld();
	FVector RayStart = ComponentToWorld.TransformPosition(LimbRootBoneCompTransform.GetLocation());
	FVector RayEnd = ComponentToWorld.TransformPosition(Eftr.OutPosition);

	FCollisionQueryParams CollisionParams(FName("PowerIKWall"), GroundCollision.TraceComplex);
	FHitResult OutHit;

	// run ray-cast
	if (World->LineTraceSingleByChannel(
		OutHit,
		RayStart,
		RayEnd,
		GroundCollision.CollisionChannel,
		CollisionParams))
	{
		if (OutHit.bBlockingHit)
		{
			// get a point, pushed back from the wall by WallOffset amount
			FVector WallOffset = RayStart - OutHit.ImpactPoint;
			// project into 2D
			WallOffset.Z = 0;
			// set to length of wall offset setting
			WallOffset.Normalize();
			WallOffset = WallOffset * GroundCollision.WallOffset;

			// offset from impact point
			FVector NewFootPosition = OutHit.ImpactPoint + WallOffset;
			// project down to the foot height
			NewFootPosition.Z = RayEnd.Z;

			// convert back to component space and store in the effector
			FTransform WorldToComponent = ComponentToWorld.Inverse();
			Eftr.OutPosition = WorldToComponent.TransformPosition(NewFootPosition);

			Foot.RayHit = true;
			Foot.RayHitPosition = OutHit.ImpactPoint;
		}
	}

	Foot.RayStart = RayStart;
	Foot.RayEnd = RayEnd;
}

void FAnimNode_PowerIK_Ground::AlignToSlope(FComponentSpacePoseContext& Output)
{
	// skip slope calculations
	if (GroundFeet.empty())
	{
		return;
	}

	// update ground settings
	Ground.Settings.StrideDirection = FVectorToVec3(GroundSlope.StrideDirection);
	Ground.Settings.MaxGroundAngle = GroundSlope.MaxGroundAngle;
	Ground.Settings.MaxNormalAngularSpeed = GroundSlope.MaxNormalAngularSpeed;

	Ground.Settings.OrientToGround = GroundSlope.OrientToGround;
	Ground.Settings.OrientToPitch = GroundSlope.OrientToPitch;
	Ground.Settings.OrientToRoll = GroundSlope.OrientToRoll;

	Ground.Settings.ScaleStride = GroundSlope.ScaleStride;
	Ground.Settings.UphillStrideScale = GroundSlope.UphillStrideScale;
	Ground.Settings.DownhillStrideScale = GroundSlope.DownhillStrideScale;
	Ground.Settings.SidehillStrideScale = GroundSlope.SidehillStrideScale;
	Ground.Settings.SidehillPushOuterFeet = GroundSlope.SidehillPushOuterFeet;

	Ground.Settings.Lean = GroundSlope.Lean;
	Ground.Settings.UphillLean = GroundSlope.UphillLean;
	Ground.Settings.DownhillLean = GroundSlope.DownhillLean;
	Ground.Settings.SidehillLean = GroundSlope.SidehillLean;

	Ground.Settings.CounterLean = GroundSlope.CounterLean;
	Ground.Settings.UphillCounterLean = GroundSlope.UphillCounterLean;
	Ground.Settings.DownhillCounterLean = GroundSlope.DownhillCounterLean;
	Ground.Settings.SidehillCounterLean = GroundSlope.SidehillCounterLean;

	Ground.Settings.MoveRoot = GroundSlope.MoveRoot;
	Ground.Settings.UphillVertOffset = GroundSlope.UphillVertOffset;
	Ground.Settings.UphillHorizOffset = GroundSlope.UphillHorizOffset;
	Ground.Settings.DownhillVertOffset = GroundSlope.DownhillVertOffset;
	Ground.Settings.DownhillHorizOffset = GroundSlope.DownhillHorizOffset;
	Ground.Settings.SidehillHorizOffset = GroundSlope.SidehillHorizOffset;
	Ground.Settings.SidehillVertOffset = GroundSlope.SidehillVertOffset;

	Ground.Settings.RotateFootToGround = GroundSlope.RotateFootToGround;
	Ground.Settings.PitchFootAmount = GroundSlope.PitchFootAmount;
	Ground.Settings.RollFootAmount = GroundSlope.RollFootAmount;

	Ground.Settings.OffsetFeetPositions = GroundSlope.OffsetFeetPositions;
	Ground.Settings.StaticFootOffset = GroundSlope.StaticFootOffset;

	// get transform of root
	FTransform RootTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(Core.Root.RefSkelIndex));
	PowerIK::Transform Root;
	Root.Position = FVectorToVec3(RootTransform.GetLocation());
	Root.Rotation = FQuatToQuat(RootTransform.GetRotation());
	PowerIK::Transform OutRootOffset;

	// storage for counter lean rotation
	Quat OutCounterLeanRotation;

	// calculate all the skeleton modifications for the slope
	Ground.CalculateGroundAlignment(
		Root,
		OutRootOffset,
		GroundFeet.data(),
		GroundFeet.size(),
		OutCounterLeanRotation,
		World->DeltaTimeSeconds);

	// apply modifications to feet effectors
	// the position of the effectors may have been scaled by stride scale
	// the rotation of the effectors may have been aligned to ground normal
	for (unsigned int i = 0; i < GroundFeet.size(); ++i)
	{
		const PowerIK::GroundFoot& Foot = GroundFeet[i];
		FPowerIKEffector& Eftr = FootEffectors[i];
		Eftr.OutPosition = Vec3ToFVector(Foot.Transform.Position);
		Eftr.OutRotation = QuatToFQuat(Foot.Transform.Rotation);
	}

	// optionally MOVE the root in response to slope
	if (Ground.Settings.MoveRoot || Ground.Settings.OrientToPitch > NEAR_ZERO)
	{
		Core.Solver->TranslateSkeletonBeforeSolve(OutRootOffset.Position);
	}

	// optionally LEAN the root in response to slope
	if (Ground.Settings.Lean || Ground.Settings.OrientToPitch > NEAR_ZERO)
	{
		Core.Solver->RotateSkeletonBeforeSolve(OutRootOffset.Rotation);
	}

	// optionally COUNTER LEAN a bone relative to the slope (usually the head)
	if (Ground.Settings.CounterLean && GroundSlope.CounterLeanBoneRefSkelIndex != INDEX_NONE)
	{
		Core.Solver->RotateBoneBeforeSolve(OutCounterLeanRotation, GroundSlope.CounterLeanBoneRefSkelIndex);
	}
}


bool FAnimNode_PowerIK_Ground::InitializeSolverData(const FBoneContainer& RequiredBones)
{
	// get input skeleton
	const FReferenceSkeleton RefSkeleton = RequiredBones.GetReferenceSkeleton();
	
	// create an effector per foot
	FootEffectors.SetNum(Feet.Num());
	GroundFeet.resize(Feet.Num());
	for (int i=0; i<Feet.Num(); ++i)
	{
		FPowerIKGroundFoot& Foot = Feet[i];
		const int32 RefSkelIndex = RefSkeleton.FindBoneIndex(Foot.BoneName);
		if (RefSkelIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Ground: Unset or invalid foot bone name: %s"), *Foot.BoneName.ToString());
			return false;
		}

		FootEffectors[i].BoneName = Foot.BoneName;
		FootEffectors[i].RotateBone = true;
		FootEffectors[i].AffectsCenterOfGravity = true;
	}

	// load counter lean bone
	GroundSlope.CounterLeanBoneRefSkelIndex = RefSkeleton.FindBoneIndex(GroundSlope.CounterLeanBoneName);
	if (GroundSlope.CounterLean && GroundSlope.CounterLeanBoneRefSkelIndex == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning, TEXT("PowerIK: Counter Lean Bone Name references unknown bone name: %s"), *GroundSlope.CounterLeanBoneName.ToString());
		return false;
	}
	
	// load skeleton into core
	Core.LoadBonesFromAnimGraph(RequiredBones);
	Core.InitializeSolver(
		CharacterRoot,
		FootEffectors,
		ExcludedBones, 
		BendDirections, 
		JointLimits);

	return Core.IsInitialized;
}

void FAnimNode_PowerIK_Ground::GatherDebugData(FNodeDebugData& DebugData)
{
	// called from command: "showdebug animation"

	// draw effector locations
	Core.DebugDrawEffectorsInEditor(DebugData, FootEffectors, DebugDrawSize);

#if WITH_EDITORONLY_DATA

	// draw the ray trace results for each foot
	UWorld* CurrentWorld = DebugData.AnimInstance->GetWorld();
	FTransform ComponentToWorld = DebugData.AnimInstance->GetSkelMeshComponent()->GetComponentToWorld();
	for (int i = 0; i < FootEffectors.Num(); ++i)
	{
		const FPowerIKGroundFoot& Foot = Feet[i];
		FColor Color = Foot.RayHit ? FColor::Green : FColor::Red;
		DrawDebugLine(CurrentWorld, Foot.RayStart, Foot.RayEnd, Color, false, -1.f, SDPG_Foreground, 1.0f);
		if (Foot.RayHit)
		{
			DrawDebugPoint(CurrentWorld, Foot.RayHitPosition, DebugDrawSize * 0.5f, FColor::Yellow, false, -1.f, SDPG_Foreground);
		}
	}

#endif

	// gather on-screen debug text
	FString DebugLine = DebugData.GetNodeName(this);
	DebugLine += "(";
	AddDebugNodeData(DebugLine);
	DebugLine += FString::Printf(TEXT(" Num Foot Effectors: %d)"), FootEffectors.Num());
	DebugData.AddDebugItem(DebugLine);
	ComponentPose.GatherDebugData(DebugData);
}

#if WITH_EDITOR
void FAnimNode_PowerIK_Ground::ConditionalDebugDraw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* MeshComp) const
{
	FTransform LocalToWorld = MeshComp->GetComponentToWorld();
	Core.DebugDrawEffectorsInPreview(PDI, DebugDrawSize, LocalToWorld, FootEffectors);
	Core.DebugDrawJointLimits(PDI, DebugDrawSize);
}
#endif // WITH_EDITOR
