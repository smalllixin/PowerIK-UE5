/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#include "PowerIK_UnrealCore.h"

#include "PowerIKRuntime/sdk/include/PowerIKSettings.h"

#include "Animation/AnimInstance.h"
#include "Animation/AnimNodeBase.h"

#include "Rigs/RigBoneHierarchy.h"

#include "DrawDebugHelpers.h"
#include "Engine/Private/Collision/CollisionDebugDrawing.h"

#include <string.h>

using PowerIK::Vec3;
using PowerIK::Quat;

Vec3 FVectorToVec3(FVector V)
{
	return Vec3(V.X, V.Y, V.Z);
}

FVector Vec3ToFVector(Vec3 V)
{
	return FVector(V.X, V.Y, V.Z);
}

Quat FQuatToQuat(FQuat Q)
{
	return Quat(Q.X, Q.Y, Q.Z, Q.W);
}

FQuat QuatToFQuat(Quat Q)
{
	return FQuat(Q.X, Q.Y, Q.Z, Q.W);
}

void CopyFNameToString(FName N, std::string& OutString)
{
	FString const NameString = N.ToString();
	OutString = StringCast<ANSICHAR>(*NameString).Get();
}


FPowerIKCore::~FPowerIKCore()
{
	if (Solver)
	{
		delete Solver;
	}
}

FPowerIKCore::FPowerIKCore(const FPowerIKCore& Other)
{
	Root = Other.Root;
	BonesData = Other.BonesData;
	BoneNameToIndexMap = Other.BoneNameToIndexMap;
	EffectorsData = Other.EffectorsData;
	Solver = nullptr;
	IsInitialized = false;
}

void FPowerIKCore::operator=(const FPowerIKCore& Other)
{
	Root = Other.Root;
	BonesData = Other.BonesData;
	BoneNameToIndexMap = Other.BoneNameToIndexMap;
	EffectorsData = Other.EffectorsData;
	Solver = nullptr;
	IsInitialized = false;
}

void FPowerIKCore::Reset()
{
	// instantiate a Power IK solver
	if (!Solver)
	{
		Solver = new PowerIK::Solver();
	}

	// reset data structures
	Solver->Reset();
	BonesData.Reset();
	EffectorsData.Reset();
}

void FPowerIKCore::LoadBonesFromAnimGraph(const FBoneContainer& RequiredBones)
{
	// reset all data structures
	Reset();
	
	// load bone names, indices and parent relations
	const FReferenceSkeleton RefSkeleton = RequiredBones.GetReferenceSkeleton();
	TArray<uint16> SkeletonBoneIndices = RequiredBones.GetBoneIndicesArray();;
	BonesData.SetNum(SkeletonBoneIndices.Num());
	for (int i = 0; i < SkeletonBoneIndices.Num(); ++i)
	{
		int32 RefSkelIndex = SkeletonBoneIndices[i];
		FPowerIKBoneData& Bone = BonesData[i];
		Bone.BoneName = RefSkeleton.GetBoneName(RefSkelIndex);
		CopyFNameToString(Bone.BoneName, Bone.NameString);
		Bone.Index = RefSkelIndex;
		Bone.ParentIndex = RequiredBones.GetParentBoneIndex(FCompactPoseBoneIndex(i)).GetInt();

		// store in the map for later lookup
		BoneNameToIndexMap.Add(Bone.BoneName, RefSkelIndex);
	}
}

void FPowerIKCore::LoadBonesFromControlRig(URigHierarchy* Hierarchy)
{
	// reset all data structures
	Reset();

	// load bone names, indices and parent relations
	BonesData.SetNum(Hierarchy->Num());
	for (int i = 0; i < Hierarchy->Num(); ++i)
	{
		const FRigBoneElement* RigBoneElem = Cast<FRigBoneElement>(Hierarchy->Get(i));
		// const FRigBone& RigBone = (*Hierarchy)[i];
		
		FPowerIKBoneData& Bone = BonesData[i];
		Bone.BoneName = RigBoneElem->GetName();
		CopyFNameToString(Bone.BoneName, Bone.NameString);
		Bone.Index = RigBoneElem->GetIndex();
		Bone.ParentIndex = RigBoneElem->ParentElement->GetIndex();

		// store in the map for later lookup
		BoneNameToIndexMap.Add(Bone.BoneName, RigBoneElem->GetIndex());
	}
}

int32 FPowerIKCore::GetBoneIndex(FName BoneName)
{
	checkSlow(BonesData.Num() == BoneNameToIndexMap.Num());
	int32 BoneIndex = INDEX_NONE;
	if (BoneName != NAME_None)
	{
		const int32* IndexPtr = BoneNameToIndexMap.Find(BoneName);
		if (IndexPtr)
		{
			BoneIndex = *IndexPtr;
		}
	}
	return BoneIndex;
}

int32 FPowerIKCore::GetCompactIndex(int32 RefSkeletonIndex)
{
	// find the compact index of the bone 
	// given the index in the reference skeleton
	for (int32 j = 0; j < BonesData.Num(); ++j)
	{
		if (BonesData[j].Index == RefSkeletonIndex)
		{
			return j;
		}
	}

	return INDEX_NONE;
}

FTransform FPowerIKCore::GetRefPoseOfBoneInComponentSpace(
	const FReferenceSkeleton& RefSkel, 
	const FName& BoneName)
{
	// init component space transform with local transform
	FTransform ComponentSpaceTransform = FTransform::Identity;

	// walk up parent chain until we reach root (ParentIndex == INDEX_NONE)
	int32 BoneIndex = RefSkel.FindBoneIndex(BoneName);
	FName CurrentBoneName;
	while (BoneIndex != INDEX_NONE)
	{
		CurrentBoneName = RefSkel.GetBoneName(BoneIndex);
		FTransform BoneLocal = RefSkel.GetRefBonePose()[BoneIndex];
		ComponentSpaceTransform = ComponentSpaceTransform * BoneLocal;
		// now move up to parent
		BoneIndex = RefSkel.GetParentIndex(BoneIndex);
	}

	return ComponentSpaceTransform;
}

bool FPowerIKCore::InitializeSolver(
	const FName CharacterRoot,
	const TArrayView<FPowerIKEffector>& Effectors, // bone indices filled out here, only need names set
	const TArrayView<FPowerIKExcludedBone>& ExcludedBones,
	const TArrayView<FPowerIKBoneBendDirection>& BendDirections,
	const TArrayView<FPowerIKBoneLimit>& JointLimits)
{
	IsInitialized = false;
	
	// load character root setting
	Root.RefSkelIndex = GetBoneIndex(CharacterRoot);
	if (Root.RefSkelIndex == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning, TEXT("PowerIK: Unset or invalid character root bone: %s"), *CharacterRoot.ToString());
		return false;
	}
	CopyFNameToString(CharacterRoot, Root.BoneName);

	// load excluded bones
	for (int i = 0; i < ExcludedBones.Num(); ++i)
	{
		int32 ExcludedBoneIndex = GetBoneIndex(ExcludedBones[i].BoneName);
		if (ExcludedBoneIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK: Excluded bone references unknown bone name: %s"), *ExcludedBones[i].BoneName.ToString());
			continue;
		}

		for (FPowerIKBoneData& BoneData : BonesData)
		{
			if (BoneData.Index == ExcludedBoneIndex)
			{
				BoneData.IsExcluded = true;
				break;
			}
		}

		FPowerIKBoneData& Bone = BonesData[ExcludedBoneIndex];
		Bone.IsExcluded = true;
	}

	// load joint limits into BoneData
	for (int i = 0; i < JointLimits.Num(); ++i)
	{
		const FPowerIKBoneLimit& JointLimit = JointLimits[i];
		bool FoundBone = false;
		for (int j = 0; j < BonesData.Num(); ++j)
		{
			FPowerIKBoneData& Bone = BonesData[j];
			if (JointLimit.BoneName == Bone.BoneName)
			{
				FoundBone = true;
				Bone.Stiffness = JointLimit.Stiffness;
				Bone.UseLimits = JointLimit.UseLimits;
				Bone.MinLimits = JointLimit.MinAngles;
				Bone.MaxLimits = JointLimit.MaxAngles;
				Bone.NegateXAxis = JointLimit.NegateXAxis;
				break;
			}
		}

		if (!FoundBone)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK: Joint Limit references unknown bone name: %s"), *JointLimit.BoneName.ToString());
		}
	}

	// load bone bending directions
	for (int i = 0; i < BendDirections.Num(); ++i)
	{
		const FPowerIKBoneBendDirection& BendDir = BendDirections[i];
		bool FoundBone = false;
		for (int j = 0; j < BonesData.Num(); ++j)
		{
			FPowerIKBoneData& Bone = BonesData[j];
			if (BendDir.BoneName == Bone.BoneName)
			{
				FoundBone = true;
				Bone.UseCustomBendDirection = true;
				Bone.BendDirection = BendDir.BendDirection;
				break;
			}
		}

		if (!FoundBone)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK: Bending direction references unknown bone name: %s"), *BendDir.BoneName.ToString());
		}
	}

	// load bone data into actual solver
	Solver->SetNumBones(BonesData.Num());
	for (int i = 0; i < BonesData.Num(); ++i)
	{
		FPowerIKBoneData Bone = BonesData[i];

		Solver->AddBone(i, Bone.NameString.c_str(), Bone.ParentIndex);

		Solver->SetBoneLimits(
			i,
			Bone.Stiffness,
			Bone.UseLimits,
			FVectorToVec3(Bone.MinLimits),
			FVectorToVec3(Bone.MaxLimits),
			Bone.NegateXAxis);

		if (Bone.IsExcluded)
		{
			Solver->ExcludeBone(i);
		}

		if (Bone.Index == Root.RefSkelIndex)
		{
			Solver->SetRootBone(i);
		}

		if (Bone.UseCustomBendDirection)
		{
			Solver->SetBoneBendDirection(i, FVectorToVec3(Bone.BendDirection));
		}
	}

	// check all effectors have valid associated bones
	// and then load them into solver
	EffectorsData.SetNum(Effectors.Num());
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		FPowerIKEffector& Eftr = Effectors[i];
		FPowerIKEffectorData& EftrData = EffectorsData[i];
		CopyFNameToString(Eftr.BoneName, EftrData.NameString);
		EftrData.RefSkelBoneIndex = GetBoneIndex(Eftr.BoneName);

		// bail out and log warning if effector is pointing to unknown bone
		if (EftrData.RefSkelBoneIndex == INDEX_NONE)
		{
			FString BoneNameStr = Effectors[i].BoneName.ToString();
			UE_LOG(LogTemp, Warning, TEXT("PowerIK: Missing effector bone: %s"), *BoneNameStr);
			return false;
		}

		// find the compact index of the bone this effector controls
		EftrData.CompactBoneIndex = GetCompactIndex(EftrData.RefSkelBoneIndex);
		// make sure affected bone is in this LOD (ie. compact pose)
		if (EftrData.CompactBoneIndex == INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning, TEXT("PowerIK: @@ effector referring to bone that was culled in this LOD: %s"), this, *Effectors[i].BoneName.ToString());
			return false;
		}

		// get pole vector bone if there is one
		if (Eftr.PoleVector.Mode == EPoleVectorModeEnum::PV_Bone)
		{
			Eftr.PoleVector.BoneIndex = GetBoneIndex(Eftr.PoleVector.BoneName);
			if (Eftr.PoleVector.BoneIndex == INDEX_NONE)
			{
				FString BoneNameStr = Effectors[i].BoneName.ToString();
				UE_LOG(LogTemp, Warning, TEXT("PowerIK: Missing effector pole vector bone: %s"), *BoneNameStr);
				return false;
			}
		}

		Solver->AddEffector(EftrData.NameString.c_str(), EftrData.SolverEffectorIndex);
	}

	IsInitialized = true;
	return true;
}

void FPowerIKCore::UpdateEffectorBoneTransformsFromAnimGraph(
	const TArrayView<FPowerIKEffector>& Effectors, 
	FComponentSpacePoseContext& Output)
{
	// update effector transforms based on incoming pose
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		// get effector bone transform in component space
		FPowerIKEffectorData& EftrData = EffectorsData[i];
		if (EftrData.SolverEffectorIndex == INDEX_NONE)
		{
			continue;
		}
		FTransform EftrBoneComponentTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(EftrData.CompactBoneIndex));

		// store it
		FPowerIKEffector& Eftr = Effectors[i];
		Eftr.BoneCompPosition = EftrBoneComponentTransform.GetLocation();
		Eftr.BoneCompRotation = EftrBoneComponentTransform.GetRotation();
	}
}

void FPowerIKCore::UpdateEffectorBoneTransformsFromControlRig(
	const TArrayView<FPowerIKEffector>& Effectors, 
	URigHierarchy* Hierarchy)
{
	// update effector transforms based on incoming pose
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		// get effector bone transform in component space
		FPowerIKEffectorData& EftrData = EffectorsData[i];
		if (EftrData.SolverEffectorIndex == INDEX_NONE)
		{
			continue;
		}
		FTransform EftrBoneComponentTransform = Hierarchy->GetGlobalTransform(EftrData.CompactBoneIndex);
		
		// store it
		FPowerIKEffector& Eftr = Effectors[i];
		Eftr.BoneCompPosition = EftrBoneComponentTransform.GetLocation();
		Eftr.BoneCompRotation = EftrBoneComponentTransform.GetRotation();
	}
}

void FPowerIKCore::UpdateEffectorTransforms(
	const TArrayView<FPowerIKEffector>& Effectors, 
	FTransform WorldToComponent)
{
	// update effector transforms based on incoming pose
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		FPowerIKEffector& Eftr = Effectors[i];
		FPowerIKEffectorData& EftrData = EffectorsData[i];

		// hasn't been added to solver
		if (EftrData.SolverEffectorIndex == INDEX_NONE)
		{
			continue;
		}

		// create effector position and rotation to feed into solver
		// solver expects everything in component space

		// put POSITION in desired space
		if (Eftr.PositionSpace.GetValue() == ES_Component)
		{
			// is already components space
			Eftr.OutPosition = Eftr.Position;
		}
		else if (Eftr.PositionSpace.GetValue() == ES_Additive)
		{
			// is additive, interpret as being offsets relative to controlled Bone
			Eftr.OutPosition = Eftr.Position + Eftr.BoneCompPosition;
		}
		else
		{
			// is world space, convert back to component space
			Eftr.OutPosition = WorldToComponent.TransformPosition(Eftr.Position);
		}

		// put ROTATION in desired space
		if (Eftr.RotationSpace.GetValue() == ES_Component)
		{
			// is already components space
			Eftr.OutRotation = Eftr.Rotation.Quaternion();
		}
		else if (Eftr.RotationSpace.GetValue() == ES_Additive)
		{
			// is additive, interpret as being offsets relative to controlled Bone
			Eftr.OutRotation = Eftr.Rotation.Quaternion() * Eftr.BoneCompRotation;
		}
		else
		{
			// is world space, convert back to component space
			Eftr.OutRotation = WorldToComponent.TransformRotation(Eftr.Rotation.Quaternion());
		}
	}
}

void FPowerIKCore::SetSolverBoneTransformsFromAnimGraph(FComponentSpacePoseContext& Output)
{
	// update bone data in solver
	for (int i = 0; i < Solver->GetNumBones(); ++i)
	{
		FTransform ComponentSpace = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(i));
		Solver->SetBoneTransform(
			i,
			FVectorToVec3(ComponentSpace.GetTranslation()),
			FQuatToQuat(ComponentSpace.GetRotation()));
	}
}

void FPowerIKCore::SetSolverBoneTransformsFromControlRig(URigHierarchy* Hierarchy)
{
	// update bone data in solver
	for (int i = 0; i < Solver->GetNumBones(); ++i)
	{
		FTransform ComponentSpace = Hierarchy->GetGlobalTransform(i);
		Solver->SetBoneTransform(
			i,
			FVectorToVec3(ComponentSpace.GetTranslation()),
			FQuatToQuat(ComponentSpace.GetRotation()));
	}
}

bool FPowerIKCore::CheckSolverNeedsReinitialized(
	int32 NumBonesInSkeleton,
	TArray<FPowerIKEffector>& Effectors) const
{
	// has the solver even been initialized yet?
	if (!IsInitialized)
	{
		return true;
	}

	// has the number of input bones changed?
	// this can happen if skeleton LOD is changed
	if (Solver->GetNumBones() != NumBonesInSkeleton)
	{
		return true;
	}

	return false;
}

void FPowerIKCore::SetSolverInputs(
	float RootRotationMultiplier,
	int32 MaxSquashIterations,
	int32 MaxStretchIterations,
	int32 MaxFinalIterations,
	bool AllowBoneTranslation,
	float SmoothingMaxSpeedMultiplier,
	float SmoothingMaxDistanceMultiplier,
	const FPowerIKCenterOfGravity& CenterOfGravity,
	const FPowerIKBodyInertia& RootInertia,
	const TArrayView<FPowerIKEffector>& Effectors)
{
	// update solver settings
	PowerIK::SolverSettings Settings;
	Settings.RootRotationMultiplier = RootRotationMultiplier;
	Settings.SquashIterations = MaxSquashIterations;
	Settings.StretchIterations = MaxStretchIterations;
	Settings.FinalIterations = MaxFinalIterations;
	Settings.WorldUpNormal = Vec3(0.0f, 0.0f, 1.0f);
	Settings.AllowBoneTranslation = AllowBoneTranslation;
	Solver->SetSolverSettings(Settings);

	// set the COG constraint parameters
	PowerIK::CenterOfGravitySettings COGSettings;
	COGSettings.Alpha = CenterOfGravity.Alpha;
	COGSettings.HorizAmount = CenterOfGravity.HorizAmount;
	COGSettings.VertAmount = CenterOfGravity.VertAmount;
	COGSettings.PullBodyAmount = CenterOfGravity.PullBodyAmount;
	Solver->SetCOGSettings(COGSettings);

	PowerIK::InertiaSettings InertiaSettings;
	InertiaSettings.ApplyInertiaToBody = RootInertia.ApplyInertiaToBody;
	InertiaSettings.SpringStrength = RootInertia.SpringStrength;
	InertiaSettings.SpringDamping = RootInertia.SpringDamping;
	Solver->SetInertiaSettings(InertiaSettings);

	// update effector data in solver
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		FPowerIKEffector& Eftr = Effectors[i];
		FPowerIKEffectorData& EftrData = EffectorsData[i];

		// hasn't been added to solver
		if (EftrData.SolverEffectorIndex == INDEX_NONE)
		{
			continue; // should never get here
		}

		// update effector transform in solver
		Solver->SetEffectorTransform(
			EftrData.SolverEffectorIndex,
			FVectorToVec3(Eftr.OutPosition),
			FQuatToQuat(Eftr.OutRotation),
			Eftr.RotateBone,
			Eftr.RotateLimb);

		// update alpha / pull weight of effector
		Solver->SetEffectorWeights(
			EftrData.SolverEffectorIndex,
			Eftr.Alpha,
			Eftr.PullWeight,
			Eftr.NormalizePulling,
			FVectorToVec3(Eftr.PositivePullFactor),
			FVectorToVec3(Eftr.NegativePullFactor));

		// update misc effector settings
		Solver->SetEffectorSettings(
			EftrData.SolverEffectorIndex,
			Eftr.DeltaSmoothSpeed,
			Eftr.AngularDeltaSmoothSpeed,
			Eftr.AffectsCenterOfGravity);

		// tell solver to smooth effector position/rotation over time
		PowerIK::EffectorSmoothingSettings SmoothSettings;
		SmoothSettings.SmoothPositionOverTime = Eftr.Smoothing.SmoothPositionOverTime;
		SmoothSettings.MaxPositionSpeed = Eftr.Smoothing.MaxPositionSpeed * SmoothingMaxSpeedMultiplier;
		SmoothSettings.MaxPositionDistance = Eftr.Smoothing.MaxPositionDistance * SmoothingMaxDistanceMultiplier;
		SmoothSettings.SmoothRotationOverTime = Eftr.Smoothing.SmoothRotationOverTime;
		SmoothSettings.MaxDegreesSpeed = Eftr.Smoothing.MaxDegreesSpeed;
		SmoothSettings.MaxDegreesDistance = Eftr.Smoothing.MaxDegreesDistance;
		Solver->SetEffectorSmoothing(EftrData.SolverEffectorIndex, SmoothSettings);

		// set pole vector settings
		PowerIK::EffectorPoleVector PoleVector;
		unsigned int PVMode = (unsigned int)Eftr.PoleVector.Mode;
		PoleVector.Mode = static_cast<PowerIK::PoleVectorMode>(PVMode);
		PoleVector.Position = FVectorToVec3(Eftr.PoleVector.Position);
		PoleVector.BoneIndex = Eftr.PoleVector.BoneIndex;
		PoleVector.AngleOffset = Eftr.PoleVector.AngleOffset;
		Solver->SetEffectorPoleVector(EftrData.SolverEffectorIndex, PoleVector);
	}
}

void FPowerIKCore::CopySolverOutputToAnimGraph(
	TArray<FBoneTransform>& OutBoneTransforms, 
	FComponentSpacePoseContext& Output)
{
	for (int i = 0; i < Solver->GetNumBones(); ++i)
	{
		// position / rotation come from solver
		Vec3 Position;
		Quat Rotation;
		Solver->GetBoneTransform(i, Position, Rotation);
		FTransform BoneTransform;
		BoneTransform.SetTranslation(Vec3ToFVector(Position));
		BoneTransform.SetRotation(QuatToFQuat(Rotation));

		// scale is passed through from input pose
		FTransform ComponentSpace = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(i));
		BoneTransform.SetScale3D(ComponentSpace.GetScale3D());

		// set transform of bone
		FCompactPoseBoneIndex CompactIndex = FCompactPoseBoneIndex(i);
		OutBoneTransforms.Add(FBoneTransform(CompactIndex, BoneTransform));
	}
}

void FPowerIKCore::CopySolverOutputToControlRig(URigHierarchy* Hierarchy)
{
	for (int i = 0; i < Solver->GetNumBones(); ++i)
	{
		Vec3 Position;
		Quat Rotation;
		Solver->GetBoneTransform(i, Position, Rotation);

		FTransform BoneTransform;
		BoneTransform.SetTranslation(Vec3ToFVector(Position));
		BoneTransform.SetRotation(QuatToFQuat(Rotation));
		Hierarchy->SetGlobalTransform(i, BoneTransform, false);
	}
}

void FPowerIKCore::DebugDrawJointLimits(FPrimitiveDrawInterface* PDI, float DebugDrawSize) const
{
	// don't draw until core is initialized
	if (!IsInitialized || Solver == nullptr)
	{
		return;
	}
	
	// draw the joint limits
	for (int i = 0; i < Solver->GetNumBones(); ++i)
	{
		Vec3 MinLimits;
		Vec3 MaxLimits;
		bool NegateXAxis;
		bool HasLimits;
		Solver->GetBoneLimits(i, MinLimits, MaxLimits, NegateXAxis, HasLimits);
		if (!HasLimits)
		{
			continue;
		}

		Vec3 Position;
		Quat Rotation;
		Solver->GetBoneTransform(i, Position, Rotation);

		FTransform BoneTransform;
		BoneTransform.SetTranslation(Vec3ToFVector(Position));
		BoneTransform.SetRotation(QuatToFQuat(Rotation));

		// for each axis
		for (int j = 0; j < 3; ++j)
		{
			EAxis::Type PrimaryAxis;
			EAxis::Type SecondaryAxis;
			FLinearColor Color;
			float NegatePrimary = 1.0f;
			float NegateSecondary = 1.0f;

			switch (j)
			{
			case 0:
				PrimaryAxis = EAxis::Y;
				SecondaryAxis = EAxis::Z;
				Color = FLinearColor::Red;
				break;
			case 1:
				PrimaryAxis = EAxis::X;
				SecondaryAxis = EAxis::Z;
				Color = FLinearColor::Green;
				NegatePrimary = NegateXAxis ? -1.0f : 1.0f;
				break;
			case 2:
				PrimaryAxis = EAxis::X;
				SecondaryAxis = EAxis::Y;
				Color = FLinearColor::Blue;
				NegatePrimary = NegateXAxis ? -1.0f : 1.0f;
				break;
			default:
				PrimaryAxis = EAxis::X;
				SecondaryAxis = EAxis::Z;
				Color = FLinearColor::Green;
				NegatePrimary = NegateXAxis ? -1.0f : 1.0f;
				break;
			}

			FVector AxisA = BoneTransform.GetUnitAxis(PrimaryAxis) * NegatePrimary;
			FVector AxisB = BoneTransform.GetUnitAxis(SecondaryAxis) * NegateSecondary;
			FVector Base = Vec3ToFVector(Position);
			float MinAngle = MinLimits[j];
			float MaxAngle = MaxLimits[j];
			float Radius = DebugDrawSize * 1.0f;
			int32 Sections = 5;

			DrawArc(PDI, Base, AxisA, AxisB, MinAngle, MaxAngle, Radius, Sections, Color, SDPG_Foreground);

			float Angle = MaxAngle - MinAngle;
			FVector SideVertexA = Base + Radius * (FMath::Cos(MinAngle * (PI / 180.0f)) * AxisA + FMath::Sin(MinAngle * (PI / 180.0f)) * AxisB);
			FVector SideVertexB = Base + Radius * (FMath::Cos(MaxAngle * (PI / 180.0f)) * AxisA + FMath::Sin(MaxAngle * (PI / 180.0f)) * AxisB);
			PDI->DrawLine(BoneTransform.GetTranslation(), SideVertexA, Color, SDPG_Foreground);
			PDI->DrawLine(BoneTransform.GetTranslation(), SideVertexB, Color, SDPG_Foreground);
		}
	}
}

void FPowerIKCore::DebugDrawEffectorsInPreview(
	FPrimitiveDrawInterface* PDI,
	float DebugDrawSize,
	const FTransform& LocalToWorld,
	const TArray<FPowerIKEffector>& Effectors) const
{
	// don't draw until core is initialized
	if (!IsInitialized || Solver == nullptr)
	{
		return;
	}
	
	// draw the effector targets
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		const FPowerIKEffector& Eftr = Effectors[i];

		// draw the target transforms
		FTransform EffectorTransform;
		FVector LocalPosition = Eftr.OutPosition;
		FVector WorldPosition = LocalToWorld.TransformPosition(LocalPosition);
		EffectorTransform.SetRotation(Eftr.Rotation.Quaternion());
		EffectorTransform.SetTranslation(WorldPosition);
		FMatrix Matrix = EffectorTransform.ToMatrixNoScale();
		DrawWireDiamond(PDI, Matrix, DebugDrawSize, FLinearColor::Yellow, SDPG_Foreground);
		FRotator EffectorRotation = Eftr.Rotation;
		DrawCoordinateSystem(PDI, WorldPosition, EffectorRotation, DebugDrawSize * 0.75f, SDPG_Foreground);
	}
}

void FPowerIKCore::DebugDrawEffectorsInEditor(
	FNodeDebugData& DebugData,
	const TArray<FPowerIKEffector>& Effectors, 
	float DebugDrawSize) const
{
	#if WITH_EDITORONLY_DATA

	// put things in world space for drawing
	UWorld* CurrentWorld = DebugData.AnimInstance->GetWorld();
	FTransform ComponentToWorld = DebugData.AnimInstance->GetSkelMeshComponent()->GetComponentToWorld();
	// draw effectors
	for (int i = 0; i < Effectors.Num(); ++i)
	{
		const FPowerIKEffector& Effector = Effectors[i];
		FVector EftrPosition = ComponentToWorld.TransformPosition(Effector.OutPosition);
		DrawDebugSphere(CurrentWorld, EftrPosition, DebugDrawSize, 2, FColor::Green, false, -1.f, SDPG_Foreground, 1.0f);
		FVector AnimatedPosition = ComponentToWorld.TransformPosition(Effector.BoneCompPosition);
		DrawDebugSphere(CurrentWorld, AnimatedPosition, DebugDrawSize * 0.5f, 2, FColor::Blue, false, -1.f, SDPG_Foreground, 1.0f);
	}
	
	#endif
}

