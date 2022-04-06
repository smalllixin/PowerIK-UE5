/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

//#pragma optimize( "", off )

#include "AnimNode_PowerIK_Walk.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include <string>
#include "PowerIKRuntime/sdk/include/PowerIKSettings.h"

#if WITH_EDITOR
#include "DrawDebugHelpers.h"
#endif

using PowerIK::Vec3;
using PowerIK::Quat;


FAnimNode_PowerIK_Walk::FAnimNode_PowerIK_Walk()
{
#if WITH_EDITOR
	static ConstructorHelpers::FObjectFinder<UCurveFloat> HeightCurveFinder(TEXT("/PowerIK/Step_Height_Curve.Step_Height_Curve"));
	StepHeightCurve = HeightCurveFinder.Object;
	check(StepHeightCurve != nullptr);

	static ConstructorHelpers::FObjectFinder<UCurveFloat> SpeedCurveFinder(TEXT("/PowerIK/Step_Speed_Curve.Step_Speed_Curve"));
	StepSpeedCurve = SpeedCurveFinder.Object;
	check(StepSpeedCurve != nullptr);
#endif
}

void FAnimNode_PowerIK_Walk::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	World = Context.AnimInstanceProxy->GetSkelMeshComponent()->GetWorld();
}

void FAnimNode_PowerIK_Walk::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
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

bool FAnimNode_PowerIK_Walk::IsValidToEvaluate(
	const USkeleton* Skeleton,
	const FBoneContainer& RequiredBones)
{
	return IsInitialized; // virtual function called by Unreal
}

void FAnimNode_PowerIK_Walk::EvaluateSkeletalControl_AnyThread(
	FComponentSpacePoseContext& Output,
	TArray<FBoneTransform>& OutBoneTransforms)
{
	SCOPE_CYCLE_COUNTER(STAT_PowerIK_Walk_Eval);

	// update the foot effectors
	CheckForTeleport(Output);
	UpdateLimbTargets(Output);
	UpdateLimbsWantToRePlant(Output);
	UpdateLimbFeet(Output);

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
		SCOPE_CYCLE_COUNTER(STAT_PowerIK_Walk_Solve);
		Core.Solver->Solve(World->DeltaTimeSeconds, SolverAlpha);
	}

	// copy the results
	Core.CopySolverOutputToAnimGraph(OutBoneTransforms, Output);
}


void FAnimNode_PowerIK_Walk::CheckForTeleport(FComponentSpacePoseContext& Output)
{
	USkeletalMeshComponent* SkeletalMesh = Output.AnimInstanceProxy->GetSkelMeshComponent();
	FVector CompWorldPosition = SkeletalMesh->GetComponentLocation();

	// check if the feet are too far away from component (pawn was teleported)
	bool DoTeleport = false;
	float MaxDist = TeleportDistance * TeleportDistance;
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];
		if ((Limb.CurrentWorldPosition - CompWorldPosition).SizeSquared() >= MaxDist)
		{
			DoTeleport = true;
			break;
		}
	}

	if (!DoTeleport)
	{
		return;
	}

	// need to teleport the feet
	FTransform ComponentToWorld = SkeletalMesh->GetComponentToWorld();
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];
		FVector FootWorldPosition = ComponentToWorld.TransformPosition(Limb.EndOrigCompPosition);
		Limb.PlantedWorldPosition = FootWorldPosition;
		Limb.CurrentWorldPosition = FootWorldPosition;
		Limb.IsPlanted = true;
		Limb.PlantedWorldPosition = FootWorldPosition;

		if (PrintTeleportWarning)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: Teleported limbs."));
		}
	}

	// update root plant location
	LastRootLocationWhenPlanted = CompWorldPosition;
}


void FAnimNode_PowerIK_Walk::UpdateLimbTargets(FComponentSpacePoseContext& Output)
{
	USkeletalMeshComponent* SkeletalMesh = Output.AnimInstanceProxy->GetSkelMeshComponent();
	// transform from character to world space
	FTransform ComponentToWorld = SkeletalMesh->GetComponentToWorld();
	FVector CurrentWorldLocation = ComponentToWorld.GetLocation();
	
	// clamp world space location of root to within 
	// StepMaxReachDistance of last planted location
	FVector LastPlantLocationToRoot = CurrentWorldLocation - LastRootLocationWhenPlanted;
	float DistanceToLastPlantedRootLocation = LastPlantLocationToRoot.Size();
	if (DistanceToLastPlantedRootLocation > StepMaxReachDistance)
	{
		FVector RootDirection = LastPlantLocationToRoot;
		RootDirection.Normalize();
		FVector ClampedWorldLocation = LastRootLocationWhenPlanted + RootDirection * StepMaxReachDistance;
		ComponentToWorld.SetLocation(ClampedWorldLocation);
		ComponentToWorld.SetTranslation(ClampedWorldLocation);
	}

	// generate target locations on ground
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];

		// get raycast origin in world space
		Limb.RayStart = ComponentToWorld.TransformPosition(Limb.StartOrigCompPosition);
		// rotate ray vector from capsule space into world space
		FVector RayDirection = ComponentToWorld.TransformVector(Limb.RayDirectionInitial);
		// calculate ray end point
		Limb.RayEnd = Limb.RayStart + (RayDirection * (Limb.LimbLength * 2.0f));

		// update target while planted
		// run ray-cast
		FCollisionQueryParams CollisionParams(FName("PowerIKWalk"), TraceComplex);
		CollisionParams.bIgnoreTouches = true;
		FHitResult OutHit;
		if (World->LineTraceSingleByChannel(
			OutHit,
			Limb.RayStart,
			Limb.RayEnd,
			CollisionChannel,
			CollisionParams))
		{
			// record that ray hit and it's location
			Limb.RayHit = true;
			Limb.TargetWorldPosition = OutHit.ImpactPoint;
			if (Limb.IsPlanted)
			{
				Limb.TargetWorldNormal = OutHit.Normal; // no updating rotation mid stride.
			}
			
		}
		else
		{
			Limb.RayHit = false;
			Limb.TargetWorldPosition = Limb.RayEnd;
			Limb.TargetWorldNormal = FVector::UpVector;
		}
		
		// move foot target so it doesn't collide with other feet
		EnforceFootPlaneConstraints(i);
		ResolveLimbTargetCollision(i);
		Limb.DistanceToTarget = FVector::Dist(Limb.TargetWorldPosition, Limb.CurrentWorldPosition);
	}
}

void FAnimNode_PowerIK_Walk::EnforceFootPlaneConstraints(int32 LimbIndex)
{
	FPowerIKWalkingLimb& FootLimb = Limbs[LimbIndex];

	FVector PlantPos = FootLimb.PlantedWorldPosition;
	FVector TgtPos = FootLimb.TargetWorldPosition;
	PlantPos.Z = 0.0f;
	TgtPos.Z = 0.0f;

	// check if limb is constrained to a particular side of another foot
	// and if so, forcibly constrain the target to remain on the correct side
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& OtherLimb = Limbs[i];

		// same foot
		if (i == LimbIndex)
		{
			continue;
		}

		const FPlaneConstraint& PlaneConstraint = FootLimb.FootPlaneConstraints[i];

		// not constrained to this foot right now
		if (!PlaneConstraint.Active)
		{
			continue;
		}

		// check if target is on right side of other foot
		FVector OtherFootPos = OtherLimb.CurrentWorldPosition;
		OtherFootPos.Z = 0.0f;
		FVector OtherFootToTarget = TgtPos - OtherFootPos;
		OtherFootToTarget.Normalize();
		if (FVector::DotProduct(PlaneConstraint.Normal, OtherFootToTarget) > 0.0f)
		{
			continue;
		}

		// target is on WRONG side of other foot, so let's simply project
		// it to a point on the other side near to the plane
		FVector FootToOtherFoot = OtherFootPos - PlantPos;
		FVector FootToOtherFootDirection = FootToOtherFoot;
		FootToOtherFootDirection.Normalize();

		FVector PlantToTarget = TgtPos - PlantPos;
		FVector PlantToTargetProjOnPlantToOtherFoot = PlantToTarget.ProjectOnToNormal(FootToOtherFootDirection);
		FVector NewTarget = PlantPos + PlantToTargetProjOnPlantToOtherFoot;
		NewTarget = NewTarget + PlaneConstraint.Normal * 0.1f;
		NewTarget.Z = FootLimb.TargetWorldPosition.Z;
		FootLimb.TargetWorldPosition = NewTarget;
	}
}

void FAnimNode_PowerIK_Walk::ResolveLimbTargetCollision(int32 LimbIndex)
{
	FPowerIKWalkingLimb& FootLimb = Limbs[LimbIndex];
	FVector FootPos = FootLimb.PlantedWorldPosition;
	FVector TgtPos = FootLimb.TargetWorldPosition;
	FootPos.Z = 0.0f;
	TgtPos.Z = 0.0f;

	FVector FootToTgt = TgtPos - FootPos;
	FVector FootToTgtDirection = FootToTgt;
	FootToTgtDirection.Normalize();

	// check if target ray hits any other feet
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& OtherLimb = Limbs[i];
		if (i == LimbIndex)
		{
			continue;
		}

		FVector OtherFootPos = OtherLimb.CurrentWorldPosition;
		OtherFootPos.Z = 0;
		FVector FootToOther = OtherFootPos - FootPos;
		FVector FootToOtherDirection = FootToOther;
		FootToOtherDirection.Normalize();

		// early out if foot moving away from other foot
		if (FVector::DotProduct(FootToTgtDirection, FootToOtherDirection) <= 0.0f)
		{
			continue;
		}

		// check how far away it will be at nearest point to other foot
		float MinDistance = OtherLimb.FootCollisionRadius + FootLimb.FootCollisionRadius;
		FVector FootToOtherProjOnFootToTgt = FootToOther.ProjectOnToNormal(FootToTgtDirection);
		FVector NearestPointOnPathToOther = FootPos + FootToOtherProjOnFootToTgt;
		FVector OtherFootToNearestPoint = NearestPointOnPathToOther - OtherFootPos;
		if (OtherFootToNearestPoint.Size() < MinDistance)
		{
			// foot is going to collide,
			// push the target perpendicular to hit direction
			FVector OtherFootToNearestPointDirection = OtherFootToNearestPoint;
			OtherFootToNearestPointDirection.Normalize();
			FVector DeltaOffset = OtherFootToNearestPointDirection * MinDistance;
			FVector PointOnSphere = OtherFootPos + DeltaOffset;
			FVector CorrectDirection = PointOnSphere - FootPos;
			CorrectDirection.Normalize();
			float TargetDistance = FootToTgt.Size();
			FVector NewTargetVector = CorrectDirection * TargetDistance;
			float Height = FootLimb.TargetWorldPosition.Z;
			FootLimb.TargetWorldPosition = FootPos + NewTargetVector;
			FootLimb.TargetWorldPosition.Z = Height;

			// we had to modify this foot's target to avoid a collision,
			// so we have to check if it's unplanted. That would mean that we
			// are committing to moving this foot to avoid hitting the other foot and
			// thus we must commit to this direction and disallow targets on the other side
			if (!FootLimb.IsPlanted)
			{
				// this foot is now committed to staying on the same side
				// of the other foot. So we record a new plane constraint
				FPlaneConstraint& PlaneConstraint = FootLimb.FootPlaneConstraints[i];
				PlaneConstraint.Active = true;
				FVector NewTargetProjOnFootToOther = NewTargetVector.ProjectOnToNormal(FootToOtherDirection);
				PlaneConstraint.Normal = NewTargetVector - NewTargetProjOnFootToOther;
				PlaneConstraint.Normal.Normalize();
			}
		}
	}
}

void FAnimNode_PowerIK_Walk::UpdateLimbsWantToRePlant(FComponentSpacePoseContext& Output)
{
	// check which feet want to replant
	int NumLimbsWantingToReplant = 0;
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];

		float MaxReplantDist = Limb.StepLengthMultiplier + Limb.StepLength;
		bool TooFarFromTarget = Limb.DistanceToTarget > MaxReplantDist;
		bool HasPlantedRelatedLimb = AtLeastOneRelatedLimbIsPlanted(Limb);

		Limb.WantsToReplant = Limb.IsPlanted && HasPlantedRelatedLimb && TooFarFromTarget;
		NumLimbsWantingToReplant += Limb.WantsToReplant ? 1 : 0;
	}

	// if multiple limbs want to replant, 
	// determine which feet *should* replant (tie breaker)
	if (NumLimbsWantingToReplant > 1)
	{
		// calculate average of all targets
		FVector AverageLimbTarget = FVector::ZeroVector;
		for (int i = 0; i < Limbs.Num(); ++i)
		{
			FPowerIKWalkingLimb& Limb = Limbs[i];
			AverageLimbTarget = AverageLimbTarget + Limb.TargetWorldPosition;
		}
		float InvNumLimbs = 1.0f / (float)Limbs.Num();
		AverageLimbTarget = AverageLimbTarget * InvNumLimbs;

		// determine which Limb is closest to average
		float MinDistToAvgTarget = MAX_flt;
		int ClosestLimb = -1;
		for (int i = 0; i < Limbs.Num(); ++i)
		{
			FPowerIKWalkingLimb& Limb = Limbs[i];
			float DistToAvgTarget = FVector::Dist(Limb.CurrentWorldPosition, AverageLimbTarget);
			if (DistToAvgTarget < MinDistToAvgTarget)
			{
				ClosestLimb = i;
				MinDistToAvgTarget = DistToAvgTarget;
			}
		}

		// force all limbs not closest to target NOT to replant (yet)
		for (int i = 0; i < Limbs.Num(); ++i)
		{
			FPowerIKWalkingLimb& Limb = Limbs[i];
			if (Limb.WantsToReplant && ClosestLimb != i)
			{
				Limb.WantsToReplant = false;
			}
		}
	}
}

void FAnimNode_PowerIK_Walk::UpdateLimbFeet(FComponentSpacePoseContext& Output)
{
	USkeletalMeshComponent* SkeletalMesh = Output.AnimInstanceProxy->GetSkelMeshComponent();
	FTransform ComponentToWorld = SkeletalMesh->GetComponentToWorld();
	FTransform WorldToComponent = ComponentToWorld.Inverse();

	// 1. un-plant feet if they "WantToReplant"
	// 2. animate unplanted foot positions / rotations
	// 3. plant them again when step is done
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];

		// check if this foot is ready to step
		if (Limb.WantsToReplant)
		{
			// unplant the foot and begin stepping
			Limb.IsPlanted = false;
			Limb.PlantedWorldPosition = Limb.CurrentWorldPosition;
			Limb.PlantedCompRotation = Limb.CurrentCompRotation;
			Limb.TimeSinceUnPlanted = 0.0f;
		}

		// move foot through the stride while stepping
		if (!Limb.IsPlanted)
		{
			AnimateLimbStep(i, WorldToComponent);

			// plant foot when step is done
			float StepDuration = FMath::Max(Limb.StepDuration * StepDurationMultiplier, 0.1f);
			bool ShouldBePlanted = Limb.TimeSinceUnPlanted > StepDuration;
			if (ShouldBePlanted)
			{
				// plant the limb
				Limb.IsPlanted = true;

				// clear all the plane constraints
				for (int p = 0; p < Limb.FootPlaneConstraints.Num(); ++p)
				{
					Limb.FootPlaneConstraints[p].Active = false;
				}

				// update last world location of root when planted
				LastRootLocationWhenPlanted = Limb.CurrentWorldPosition - ComponentToWorld.TransformVector(Limb.EndOrigCompPosition);
			}
		}

		// set effector OutPosition / Rotation based on step animation
		FPowerIKEffector& Eftr = FootEffectors[i];
		Eftr.OutPosition = WorldToComponent.TransformPosition(Limb.CurrentWorldPosition);
		Eftr.OutRotation = Limb.CurrentCompRotation;
	}	
}

void FAnimNode_PowerIK_Walk::AnimateLimbStep(int32 LimbIndex, const FTransform& WorldToComponent)
{
	FPowerIKWalkingLimb& Limb = Limbs[LimbIndex];

	// how far through the step are we?
	float StepDuration = FMath::Max(Limb.StepDuration * StepDurationMultiplier, 0.1f);
	float PercentOfStep = Limb.TimeSinceUnPlanted / StepDuration;
	if (StepSpeedCurve != NULL)
	{
		// non-linear step speed from custom curve
		PercentOfStep = StepSpeedCurve->GetFloatValue(PercentOfStep);
	}

	// lerp from start of step to current target
	Limb.CurrentWorldPosition = FMath::Lerp(Limb.PlantedWorldPosition, Limb.TargetWorldPosition, PercentOfStep);

	// set height of foot during replant, applying global modifier to foot height
	float StepHeightFalloff = 1.0f;
	if (StepHeightCurve != NULL)
	{
		// modulate replant height using custom curve
		StepHeightFalloff = StepHeightCurve->GetFloatValue(PercentOfStep);
	}
	float HeightAdjustment = Limb.StepHeight * FMath::Max(StepHeightMultiplier, 0.0f) * StepHeightFalloff;
	Limb.CurrentWorldPosition.Z += HeightAdjustment;

	// rotate the foot through the step
	if (Limb.RotateFootToGround > 0.001f)
	{
		// blend rotation of foot from planted rotation to target ground rotation
		FVector TargetCompNormal = WorldToComponent.GetRotation().RotateVector(Limb.TargetWorldNormal);
		FQuat TargetCompGroundRotation = FQuat::FindBetweenNormals(FVector::UpVector, TargetCompNormal);
		FQuat TargetCompFootRotation = TargetCompGroundRotation * Limb.EndOrigCompRotation;
		FQuat BlendedComponentFootRotation = FQuat::FastLerp(Limb.PlantedCompRotation, TargetCompFootRotation, PercentOfStep);
		BlendedComponentFootRotation.Normalize();
		FQuat FinalFootRotation = FQuat::FastLerp(Limb.EndOrigCompRotation, BlendedComponentFootRotation, Limb.RotateFootToGround);
		FinalFootRotation.Normalize();
		Limb.CurrentCompRotation = FinalFootRotation;
	}
	else
	{
		// just keep ref pose rotation
		Limb.CurrentCompRotation = Limb.EndOrigCompRotation;
	}

	// update time in stride
	Limb.TimeSinceUnPlanted += World->DeltaTimeSeconds;
}

bool FAnimNode_PowerIK_Walk::AtLeastOneRelatedLimbIsPlanted(const FPowerIKWalkingLimb& Limb)
{
	if (Limb.RelatedLimbs.Num() == 0)
	{
		return true;
	}

	for (int i = 0; i < Limb.RelatedLimbs.Num(); ++i)
	{
		const uint8 RelatedLimbIndex = Limb.RelatedLimbs[i];
		const bool IndexInRange = RelatedLimbIndex >= 0 && RelatedLimbIndex <= Limbs.Num() - 1;
		if (!IndexInRange)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: related limb index out of range."));
			return true;
		}
		const FPowerIKWalkingLimb& RelatedLimb = Limbs[RelatedLimbIndex];
		if (RelatedLimb.IsPlanted)
		{
			return true;
		}
	}

	return false;
}

bool FAnimNode_PowerIK_Walk::InitializeSolverData(const FBoneContainer& RequiredBones)
{
	// get input skeleton
	const FReferenceSkeleton RefSkeleton = RequiredBones.GetReferenceSkeleton();

	// get local transforms of ref pose
	TArray<FTransform> RefPose = RefSkeleton.GetRefBonePose();
	
	// create an effector per foot
	FootEffectors.SetNum(Limbs.Num());
	for (int i=0; i< Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];

		Limb.StartBoneRefIndex = RefSkeleton.FindBoneIndex(Limb.StartBone);
		if (Limb.StartBoneRefIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: Unset or invalid start bone name: %s"), *Limb.StartBone.ToString());
			return false;
		}

		Limb.EndBoneRefIndex = RefSkeleton.FindBoneIndex(Limb.EndBone);
		if (Limb.EndBoneRefIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: Unset or invalid end bone name: %s"), *Limb.EndBone.ToString());
			return false;
		}

		// calculate initial length of limb
		Limb.LimbLength = GetLengthOfLimb(RefSkeleton, Limb.StartBone, Limb.EndBone);

		// make a decent guess at the base stride length,
		// this is scaled at runtime by the Limb.StrideLengthMultiplier
		Limb.StepLength = Limb.LimbLength * Limb.LimbMaxLengthMultiplier * 0.75f;

		// calculate initial ray vector based on ref pose
		FTransform StartBoneCompTransform = Core.GetRefPoseOfBoneInComponentSpace(RefSkeleton, Limb.StartBone);
		FTransform EndBoneCompTransform = Core.GetRefPoseOfBoneInComponentSpace(RefSkeleton, Limb.EndBone);
		FVector StartCompPosition = StartBoneCompTransform.GetLocation();
		FVector EndCompPosition = EndBoneCompTransform.GetLocation();
		
		// initialize ray direction
		Limb.RayDirectionInitial = EndCompPosition - StartCompPosition;
		Limb.RayDirectionInitial.Normalize();

		// initialize positions
		Limb.StartOrigCompPosition = StartCompPosition;
		Limb.EndOrigCompPosition = EndCompPosition;
		Limb.PlantedWorldPosition = EndCompPosition;
		Limb.CurrentWorldPosition = EndCompPosition;
		// initial foot rotation
		Limb.EndOrigCompRotation = EndBoneCompTransform.GetRotation();
		Limb.CurrentCompRotation = Limb.EndOrigCompRotation;
		Limb.PlantedCompRotation = Limb.EndOrigCompRotation;
		// intialize plane constraints
		Limb.FootPlaneConstraints.SetNum(Limbs.Num());

		// initialize effector settings
		FootEffectors[i].BoneName = Limb.EndBone;
		FootEffectors[i].BoneCompPosition = EndCompPosition;
		// orient to ground
		FootEffectors[i].RotateBone = true; 
	}
	
	// load skeleton into core
	Core.LoadBonesFromAnimGraph(RequiredBones);
	Core.InitializeSolver(
		CharacterRoot,
		FootEffectors,
		ExcludedBones, 
		BendDirections, 
		JointLimits);

	// now that bones are loaded, get the compact indices (in case they were LOD'd out)
	for (int i = 0; i < Limbs.Num(); ++i)
	{
		FPowerIKWalkingLimb& Limb = Limbs[i];

		Limb.StartBoneCompactIndex = Core.GetCompactIndex(Limb.StartBoneRefIndex);
		if (Limb.StartBoneCompactIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: Bone was LOD'd out: %s"), *Limb.StartBone.ToString());
			return false;
		}

		Limb.EndBoneCompactIndex = Core.GetCompactIndex(Limb.EndBoneRefIndex);
		if (Limb.EndBoneCompactIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: Bone was LOD'd out: %s"), *Limb.EndBone.ToString());
			return false;
		}
	}

	return Core.IsInitialized;
}

float FAnimNode_PowerIK_Walk::GetLengthOfLimb(
	const FReferenceSkeleton& RefSkel,
	const FName StartBone, 
	const FName EndBone)
{
	// get local transforms of ref pose
	TArray<FTransform> RefPose = RefSkel.GetRefBonePose();

	// walk up parent chain until we reach start bone or root
	int32 BoneIndex = RefSkel.FindBoneIndex(EndBone);
	FVector PrevPos = RefPose[BoneIndex].GetLocation();
	FName CurrentBoneName;
	float Distance = 0.0f;
	while (BoneIndex != INDEX_NONE)
	{
		// add the length of this bone 
		FVector Position = RefSkel.GetRefBonePose()[BoneIndex].GetLocation();
		Distance += (Position - PrevPos).Size();

		// check if we're at the root of the chain
		CurrentBoneName = RefSkel.GetBoneName(BoneIndex);
		if (CurrentBoneName == StartBone)
		{
			break;
		}

		// now move up to parent
		BoneIndex = RefSkel.GetParentIndex(BoneIndex);

		// if we hit root before the end bone, then warn the user it's configured incorrectly
		if (BoneIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK Walk: Limb End Bone is not a child of Start Bone: %s"), *EndBone.ToString());
		}
	}

	return Distance;
}

void FAnimNode_PowerIK_Walk::GatherDebugData(FNodeDebugData& DebugData)
{
	// called from command: "showdebug animation"

	// draw effector locations
	Core.DebugDrawEffectorsInEditor(DebugData, FootEffectors, DebugDrawSize);
	
#if WITH_EDITORONLY_DATA

	// put things in world space for drawing
	UWorld* CurrentWorld = DebugData.AnimInstance->GetWorld();
	FTransform ComponentToWorld = DebugData.AnimInstance->GetSkelMeshComponent()->GetComponentToWorld();

	for (int i = 0; i < Limbs.Num(); ++i)
	{
		const FPowerIKWalkingLimb& Limb = Limbs[i];

		// draw the raycast
		FColor Color = Limb.RayHit ? FColor::Green : FColor::Red;
		DrawDebugLine(CurrentWorld, Limb.RayStart, Limb.RayEnd, Color, false, -1.f, SDPG_Foreground, 1.0f);
		// draw the target position
		DrawDebugPoint(CurrentWorld, Limb.TargetWorldPosition, DebugDrawSize * 0.5f, FColor::Yellow, false, -1.f, SDPG_Foreground);
		// draw the planted position
		DrawDebugPoint(CurrentWorld, Limb.PlantedWorldPosition, DebugDrawSize * 0.5f, FColor::Red, false, -1.f, SDPG_Foreground);
		// draw collision radius
		DrawDebugCircle(CurrentWorld, Limb.CurrentWorldPosition, Limb.FootCollisionRadius, 8, FColor::Red, false, -1.f, SDPG_Foreground);
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
void FAnimNode_PowerIK_Walk::ConditionalDebugDraw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* MeshComp) const
{
	FTransform LocalToWorld = MeshComp->GetComponentToWorld();
	Core.DebugDrawEffectorsInPreview(PDI, DebugDrawSize, LocalToWorld, FootEffectors);
	Core.DebugDrawJointLimits(PDI, DebugDrawSize);
}
#endif // WITH_EDITOR
