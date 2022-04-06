/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
//#pragma optimize("", off)

#include "PowerIKCore.h"

#include <algorithm>
#include <cmath>

namespace PowerIK
{
	using PowerIK::Core;
	using PowerIK::Bone;
	using PowerIK::Effector;
	using PowerIK::BendConstraint;
	using PowerIK::SubChain;
	using PowerIK::Quat;
	using PowerIK::Vec3;

void Core::Solve(const float InDeltaTime, const float InAlpha)
{
	DeltaTime = InDeltaTime;
	SolverAlpha = InAlpha;

	if (Effectors.empty())
	{
		return; // don't do anything until an effector is created
	}

	if (!IsInitialized)
	{
		Initialize(); // make sure solver is initialized before running first solve
	}

	if (SolverAlpha <= NEAR_ZERO)
	{
		return; // solver disabled
	}
	
	CacheInputState();				// cache inputs so solver can refer to input pose of bones and effectors
	ApplyPreSolveOffsets();			// apply modifications set from Translate/RotateSkeletonBeforeSolve()
	CacheStartPose();				// record pose before any solving begins

	SmoothEffectorsWithAlpha();		// take into consideration "alpha" blend param on effectors
	SmoothlyMoveSubRoots();			// use per-bone effector weights to move sub-chains towards effectors
	ConstrainCOG();					// apply COG constraint (optional)
	RotateSubChainRoots();			// rotate sub-roots to point at effectors

	// pull the subroots out towards effectors
	for (unsigned int I = 0; I < MainSettings.StretchIterations; ++I)
	{
		SolveSubRootPullConstraints();
	}

	UpdateInertia();				// apply inertia to un-affected sub-roots
	SquashSubChains();				// squash/stretch intermediate bones
	
	// precondition sub-chains with distance and bending constraints
	// this will give more natural / pleasing results with fewer FABRIK iterations
	for (unsigned int i = 0; i < MainSettings.SquashIterations; ++i)
	{
		SolveSubChainDistanceConstraints();
		SolveSubChainBendingConstraints(i);
	}
	
	// run multiple FABRIK iterations
	for (unsigned int i = 0; i < MainSettings.FinalIterations; ++i)
	{
		SolveFABRIKBackward();
		SolveFABRIKForward();
	}
	
	UpdateSolvedBoneRotations();				// update orientations of solved bones
	UpdatePoleVectorRotations();				// apply pole vector rotations
	UpdateNonSolvedSkeletonBelowBone(Bones[0]);	// update non-solved (kinematic) joints
}

void Core::CacheInputState()
{
	// cache orig pos/rot of bones and calculate local transforms
	for (unsigned int I = 0; I < Bones.size(); ++I)
	{
		Bone& B = Bones[I];

		if (B.Parent)
		{
			B.LocalRotationFromPose = B.Parent->Rotation.Inverse() * B.Rotation;
			B.LocalPositionFromPose = B.Parent->Rotation.Inverse().Rotate(B.Position - B.Parent->Position);
		}
		else
		{
			// root's local transform is world space
			B.LocalRotationFromPose = B.Rotation;
			B.LocalPositionFromPose = B.Position;
		}
	}

	// calculate Squash/Stretch data for sub chains
	// this cannot be pre-calculated because we want to use the input
	// pose to inform the squash shape of the sub-chains
	for (unsigned int I = 0; I < SubChains.size(); ++I)
	{
		SubChain& Chain = SubChains[I];
		if (Chain.Bones.size() <= 2)
		{
			continue; // no intermediate bones in sub chain
		}

		// clear data from previous frame
		Chain.BoneParams.clear();
		Chain.BoneDistancesToChain.clear();
		Chain.BonesRelativeToStart.clear();
		Chain.BonesRelativeToEnd.clear();

		// recalculate squash data using input pose
		Bone* ChainEnd = Chain.Bones[0];
		Bone* ChainStart = Chain.SquashStartBone;
		Vec3 LineOrigin = ChainStart->Position;
		Vec3 LineEnd = ChainEnd->Position;
		Vec3 LineDirection = LineEnd - LineOrigin;
		float LineLength = std::max<float>(0.1f, LineDirection.Length());
		LineDirection.NormalizeSafe();
		Chain.LineLengthOrig = LineLength;
		for (unsigned int k = 1; k < Chain.Bones.size() - 1; ++k)
		{
			Bone* Bone = Chain.Bones[k];
			Vec3 LineOriginToBone = Bone->Position - LineOrigin;
			float DistanceOnChainAxis = Vec3::Dot(LineOriginToBone, LineDirection);
			// calculate normalized parameter for Bone along this sub chain
			float Param = 1.0f - DistanceOnChainAxis / LineLength;
			Chain.BoneParams.push_back(Param);
			// calculate distance from bone to the chain axis
			Vec3 PointOnLine = LineOrigin + LineDirection * DistanceOnChainAxis;
			float DistanceToChain = (Bone->Position - PointOnLine).Length();
			Chain.BoneDistancesToChain.push_back(DistanceToChain);
			// calculate local position of Bone relative to start/end of sub chain
			Vec3 LocalPositionStart = ChainStart->Rotation.Inverse().Rotate(Bone->Position - ChainStart->Position);
			Vec3 LocalPositionEnd = ChainEnd->Rotation.Inverse().Rotate(Bone->Position - ChainEnd->Position);
			Chain.BonesRelativeToStart.push_back(LocalPositionStart);
			Chain.BonesRelativeToEnd.push_back(LocalPositionEnd);
		}

		// initialize bending constraints based on input pose
		for (int K = 0; K < (int)Chain.BendingConstraints.size() - 1; ++K)
		{
			Chain.BendingConstraints[K].UpdateInputState();
		}
	}

	// optionally allow translation/stretching of bone chains
	if (MainSettings.AllowBoneTranslation)
	{
		// update Bone lengths
		for (unsigned int i = 0; i < Bones.size(); ++i)
		{
			Bone& B = Bones[i];
			if (!B.Parent)
			{
				continue;
			}

			B.DistToParent = (B.Position - B.Parent->Position).Length();
		}

		// update Chain lengths
		for (unsigned int i = 0; i < SubChains.size(); ++i)
		{
			SubChain& Chain = SubChains[i];
			Chain.ChainLength = 0.0f;
			for (unsigned int j = 0; j < Chain.Bones.size(); ++j)
			{
				Bone* B = Chain.Bones[j];

				if (B->IsSolverRoot)
				{
					continue; // zero length 
				}

				if (B == Chain.SquashStartBone)
				{
					break; // chain length stops at squash bone as far as pulling is concerned
				}

				Chain.ChainLength += B->DistToParent;
			}
		}
	}

	// normalize weights each tick iff PullWeight has been changed
	NormalizeEffectorWeights();
}

void Core::CacheStartPose()
{
	// cache orig pos/rot of bones and calculate local transforms
	for (unsigned int I = 0; I < Bones.size(); ++I)
	{
		Bone& B = Bones[I];
		B.PositionFromPose = B.Position;
		B.RotationFromPose = B.Rotation;
	}
}

void Core::NormalizeEffectorWeights()
{
	if (!ReNormalizeEffectorWeights)
	{
		return; // if PullWeight updated, we need to re-normalize weights
	}

	ReNormalizeEffectorWeights = false;

	for (int I = 0; I < (int)SubChains.size(); ++I)
	{
		SubChain& Chain = SubChains[I];

		// subroots with an attached effector are 
		// 100% weighted to their effector and 0% to ALL other effectors.
		if (Chain.Bones[0]->AttachedEffector)
		{
			for (int J = 0; J < (int)Chain.EffectorDistances.size(); ++J)
			{
				float Weight = Chain.Bones[0] == Effectors[J].Bone ? 1.0f : 0.0f;
				Chain.EffectorDistWeights[J] = Weight;
				Chain.EffectorPullWeights[J] = Weight;
			}
			continue;
		}

		if (Chain.EffectorDistances.size() == 1)
		{
			// with just 1 effector, our normalized scheme will produce NANs
			// because MaxDistanceBetweenBones - EffectorWeights[0] == 0
			// which makes TotalWeight == 0, and a results in a division by 0
			Chain.EffectorDistWeights[0] = 1.0f;
			Chain.EffectorPullWeights[0] = 1.0f;
			continue;
		}

		// set weights to the inverse distances scaled by PullWeight
		float TotalDistWeight = 0.0f;
		float TotalPullWeight = 0.0f;
		for (int J = 0; J < (int)Chain.EffectorDistances.size(); ++J)
		{
			float InvDist = Chain.EffectorDistances[J];
			float InvDistScaledByPullWeight = InvDist * Effectors[J].PullWeight;
			TotalDistWeight += InvDist;
			TotalPullWeight += InvDistScaledByPullWeight;
			Chain.EffectorDistWeights[J] = InvDist;
			Chain.EffectorPullWeights[J] = InvDistScaledByPullWeight;
		}

		// no effector weights, avoid division by zero and disable smooth weighting effectors
		if (TotalDistWeight <= 0.01f || TotalPullWeight <= 0.01f)
		{
			continue;
		}

		// normalize 0-1
		const float InvTotalDistWeight = 1.0f / TotalDistWeight;
		const float InvTotalPullWeight = 1.0f / TotalPullWeight;
		for (int J = 0; J < (int)Chain.EffectorDistances.size(); ++J)
		{
			Chain.EffectorDistWeights[J] *= InvTotalDistWeight;
			Chain.EffectorPullWeights[J] *= InvTotalPullWeight;
		}
	}
}

void Core::SmoothEffectorsWithAlpha()
{
	for (unsigned int I = 0; I < Effectors.size(); ++I)
	{
		Effector& Eftr = Effectors[I];

		// smooth the POSITION DELTA of the effector over time
		Vec3 PositionDelta = Eftr.Position - Eftr.Bone->PositionFromPose;
		if (Eftr.DeltaSmoothSpeed > 0.001f)
		{
			PositionDelta = MoveTowards(
				Eftr.PrevPositionDelta,
				PositionDelta,
				Eftr.DeltaSmoothSpeed,
				1000.0f,
				DeltaTime);

			Eftr.Position = Eftr.Bone->PositionFromPose + PositionDelta;
		}
		Eftr.PrevPositionDelta = PositionDelta;

		// smooth the ROTATION DELTA of the effector over time
		Quat RotationDelta = Eftr.Bone->RotationFromPose * Eftr.Rotation.Inverse();
		if (Eftr.AngularDeltaSmoothSpeed > 0.001f)
		{
			RotationDelta = RotateTowards(
				Eftr.PrevRotationDelta,
				RotationDelta,
				Eftr.AngularDeltaSmoothSpeed,
				DeltaTime);

			Eftr.Rotation = Eftr.Bone->RotationFromPose * RotationDelta;
		}
		Eftr.PrevRotationDelta = RotationDelta;

		// POSITION temporal smoothing towards target position
		if (Eftr.SmoothSettings.SmoothPositionOverTime)
		{
			Eftr.Position = MoveTowards(
				Eftr.PrevPosition,
				Eftr.Position,
				Eftr.SmoothSettings.MaxPositionSpeed,
				Eftr.SmoothSettings.MaxPositionDistance,
				DeltaTime);
		}
		Eftr.PrevPosition = Eftr.Position;

		// ROTATION temporal smoothing towards target quaternion
		if (Eftr.SmoothSettings.SmoothRotationOverTime)
		{
			Eftr.Rotation = RotateTowards(
				Eftr.PrevRotation,
				Eftr.Rotation,
                Eftr.SmoothSettings.MaxDegreesSpeed,
				DeltaTime);
                //Eftr.SmoothSettings.MaxDegreesDistance); TODO implement rotation clamping
		}
		Eftr.PrevRotation = Eftr.Rotation;

		// lerp towards input positions by alpha
		const float Alpha = Eftr.Alpha * SolverAlpha;
		Eftr.Position = Vec3::Lerp(Eftr.Bone->PositionFromPose, Eftr.Position, Alpha);
		Eftr.Rotation = Quat::Lerp(Eftr.Bone->RotationFromPose, Eftr.Rotation, Alpha);

		// update effector deltas
		Eftr.PositionDelta = Eftr.Position - Eftr.Bone->PositionFromPose;
		Eftr.RotationDelta = Eftr.Rotation * Eftr.Bone->RotationFromPose.Inverse();
	}
}

void Core::SmoothlyMoveSubRoots()
{
	// move bones according to their effector weights
	for (unsigned int I = 0; I < SubChains.size(); ++I)
	{
		SubChain& Chain = SubChains[I];
		Bone* SubRoot = Chain.Bones[0];

		if (SubRoot->AttachedEffector)
		{
			SubRoot->Position = SubRoot->AttachedEffector->Position;
			continue;
		}

		// pull positions towards other effectors
		Vec3 PosDelta = Vec3::Zero();
		for (unsigned int J = 0; J < Effectors.size(); ++J)
		{
			Effector& Eftr = Effectors[J];

			// normalized smooth weighted positional offset accumulated
			// from each Effector with weights proportional to Effector.PullWeight
			float Weight;
			if (Eftr.NormalizePulling)
			{
				// these weights are normalized across ALL effectors
				Weight = Chain.EffectorPullWeights[J];
			}
			else
			{
				// these weights falloff proportional to distance along chain, 
				// but they are NOT normalized with respect to other effectors
				Weight = Chain.EffectorDistWeights[J] * Eftr.PullWeight;
			}

			// scale offset by directional pull weights
			Vec3 Offset = ScaleDeltaByEffectorPullWeights(Eftr.PositionDelta, Weight, Eftr);

			// accumulate offsets from each effector
			PosDelta += Offset;
		}

		// apply the final delta
		SubRoot->Position += PosDelta;
	}

	// update subroot children (after they've been moved around) for accurate pull constraints
	for (unsigned int i = 0; i < SubChains.size(); ++i)
	{
		Bone* SubRoot = SubChains[i].Bones[0];
		SetChildrenToLocalPositionsFromPose(SubRoot);
	}
}

Vec3 Core::ScaleDeltaByEffectorPullWeights(
	const Vec3& Delta,
	const float& EffectorWeight,
	const Effector& Eftr)
{
	Vec3 Offset = Delta;

	// scale entire offset
	Offset *= EffectorWeight;

	// factor-in non-linear directional weights
	Offset.X *= Offset.X > 0.0f ? Eftr.PositivePullFactor.X : Eftr.NegativePullFactor.X;
	Offset.Y *= Offset.Y > 0.0f ? Eftr.PositivePullFactor.Y : Eftr.NegativePullFactor.Y;
	Offset.Z *= Offset.Z > 0.0f ? Eftr.PositivePullFactor.Z : Eftr.NegativePullFactor.Z;

	return Offset;
}

void Core::ConstrainCOG()
{
	if (COGSettings.Alpha <= 0.001f)
	{
		return;
	}

	for (unsigned int I = 0; I < Bones.size(); ++I)
	{
		Bone& B = Bones[I];
		if (!B.IsSolverRoot)
		{
			continue;
		}

		// COG constraint, constrains COGBone (solver root) to stay near vertical
		// line passing through the center of gravity.
		//
		// "Center of Gravity" defined as the blended positions of the feet
		// OR if no feet provided, then the input pose of the COGBone

		// calculate center of gravity position
		Vec3 CenterOfGravity = Vec3::Zero();

		// calculate local space position of COG bone relative to all foot effectors
		unsigned int NumFeet = 0;
		for (int J = 0; J < (int)Effectors.size(); ++J)
		{
			Effector& Eftr = Effectors[J];
			if (Eftr.AffectsCenterOfGravity)
			{
				Vec3 EftrToCOGFromPose = B.PositionFromPose - Eftr.Bone->PositionFromPose;
				CenterOfGravity += Eftr.Bone->PositionFromPose + Eftr.PositionDelta + EftrToCOGFromPose;
				NumFeet += 1;
			}
		}

		if (NumFeet == 0)
		{
			return;
		}

		// TODO make this respect the gravity vector

		// average foot relative positions
		const float InvNumFeet = 1.0f / (float)NumFeet;
		CenterOfGravity *= InvNumFeet;
		// foot center of gravity is perpendicular to gravity (2d)
		CenterOfGravity.Z = B.Position.Z;
		// delta from center of gravity to solver root after effector weighting
		const Vec3 PosDelta = B.Position - CenterOfGravity;

		// split into parallel and perpendicular vectors relative to gravity
		const Vec3 HorizDelta = Vec3(PosDelta.X, PosDelta.Y, 0.f);
		const float HorizOffset = HorizDelta.Length();
		// push horizontally in opposite direction
		Vec3 COGDelta = HorizDelta * -COGSettings.HorizAmount;
		// compensate with vertical offset proportional to horizontal counter
		COGDelta.Z = PosDelta.Z + HorizOffset * COGSettings.VertAmount;
		const Vec3 COGPositionOffset = Vec3::Lerp(PosDelta, COGDelta, COGSettings.Alpha);
		B.Position += COGPositionOffset;

		// update fork children positions
		// these are locked relative to fork, and must be updated before
		// squashing / stretching of intermediate bones for accurate squash
		SetChildrenToLocalPositionsFromPose(&B);

		// pull neighbors along with COG
		SubChain* Chain = B.ChildSubChain;
		for (unsigned int K = 0; K < Chain->NeighborChains.size(); ++K)
		{
			Bone* NeighborTip = Chain->NeighborChains[K]->Bones[0];
			if (NeighborTip->AttachedEffector)
			{
				continue;
			}
			NeighborTip->Position += COGPositionOffset * COGSettings.PullBodyAmount;
			SetChildrenToLocalPositionsFromPose(NeighborTip);
		}

		break;
	}
}

void Core::UpdateInertia()
{
	if (!Inertia.ApplyInertiaToBody)
	{
		return;
	}

	for (int d = (int)SubChains.size() - 1; d >= 0; --d)
	{
		SubChain& Chain = SubChains[d];

		Bone* ChainRoot = Chain.Bones[Chain.Bones.size()-1];
		if (ChainRoot->AttachedEffector)
		{
			continue;
		}

		if (Inertia.UseSpring)
		{
			float Damping = 1.0f - std::max(Inertia.SpringDamping, NEAR_ZERO);
			Vec3 A = (ChainRoot->Position - Chain.Position) * std::max(Inertia.SpringStrength, NEAR_ZERO);
			Vec3 V = Chain.Velocity + A * DeltaTime;
			V = V * Damping;
			Vec3 NewPos = Chain.Position + V * DeltaTime;
			Chain.Velocity = (NewPos - Chain.Position) / std::max(DeltaTime, NEAR_ZERO);
			ChainRoot->Position = NewPos;
			Chain.Position = NewPos;
		}
		else
		{
			float SmoothFactor = std::max(Inertia.SmoothFactor, 1.0f);
			ChainRoot->Position = MoveTowardsSmooth(Chain.Position, ChainRoot->Position, SmoothFactor);
			Chain.Position = ChainRoot->Position;
		}

		if (ChainRoot->IsSolvedFork)
		{
			SetChildrenToLocalPositionsFromPose(ChainRoot);
		}
	}
}

void Core::ApplyPreSolveOffsets()
{
	// apply pre-solve global skeleton offsets
	// 
	// these move all the un-affected sub roots by a constant offset
	// this is an optional offset provided by user of the SDK and
	// is provided as a convenience to bundle procedural skeleton modification
	// into the same pass as the main IK solve (ie slope adjustment leaning)
	if (ApplySkeletonOffsets)
	{
		// translate and rotate root
		Root->Position += PreSolveSkeletonPositionOffset;
		Root->Rotation = PreSolveSkeletonRotationOffset * Root->Rotation;
		Root->Rotation.Normalize();
		
		// propagate to children
		for (int i = 0; i < (int)Root->AllChildren.size(); ++i)
		{
			Bone* Child = Root->AllChildren[i];

			//if (!Child->IsSolved)
			//{
			//	continue;
			//}

			if (!Child->Parent)
			{
				continue;
			}

			Bone* Parent = Child->Parent;
			Child->Position = Parent->Position + Parent->Rotation.Rotate(Child->LocalPositionFromPose);
			Child->Rotation = Parent->Rotation * Child->LocalRotationFromPose;
		}


		/*
		for (unsigned int I = 0; I < SubChains.size(); ++I)
		{
			Bone* SubRoot = SubChains[I].Bones[0];
			if (SubRoot->AttachedEffector)
			{
				continue;
			}

			SubRoot->Position += PreSolveSkeletonPositionOffset;
			SubRoot->Rotation = PreSolveSkeletonRotationOffset * SubRoot->Rotation;
			SubRoot->Rotation.Normalize();
		}

		
		*/
		ApplySkeletonOffsets = false; // not applied unless asked for again
	}

	// apply pre-solve rotations
	//
	// these are rotation offsets provided by user of SDK to simply
	// rotate a bone in the skeleton relative to it's input pose
	for (unsigned int I = 0; I < Bones.size(); ++I)
	{
		Bone& B = Bones[I];
		if (B.ApplyPreSolveRotationOffset)
		{
			B.Rotation = B.PreSolveRotationOffset * B.Rotation;
			B.Rotation.Normalize();

			// bone may not be part of solver, so update it's local rotation
			// non-solved bone are parented into the solved hierarchy at the end of Solve()
			if (!B.IsSolved && B.Parent)
			{
				B.LocalRotationFromPose = B.Parent->RotationFromPose.Inverse() * B.Rotation;
			}
		}
	}
}

void Core::SquashSubChains()
{
/*
	// update subroot children (after they've been moved around) for accurate squashing
	for (unsigned int i = 0; i < SubChains.size(); ++i)
	{
		SubChain& Chain = SubChains[i];
		SetChildrenToLocalPositionsFromPose(Chain.Bones[0]);
	}*/

	// iterate over all sub-chains and squash intermediate bones
	for (int i = (int)SubChains.size() - 1; i >= 0; --i)
	{
		SubChain& Chain = SubChains[i];
		if (Chain.Bones.size() <= 2)
		{
			continue; // no intermediate Bones to squash
		}

		Bone* ChainStart = Chain.SquashStartBone;
		Bone* ChainEnd = Chain.Bones[0];

		// calc rotations for start/end of chains to rotate towards each other
		Vec3 StartToEnd = (ChainEnd->Position - ChainStart->Position).Normalized();
		Vec3 StartToEndOrig = (ChainEnd->PositionFromPose - ChainStart->PositionFromPose).Normalized();
		Quat RotateStartToEnd = Quat::FromToRotation(StartToEndOrig, StartToEnd);

		// if effector is meant to rotate limb, apply it's rotation as well
		Quat ChainStartRot;
		Quat ChainEndRot;
		if (ChainStart->AttachedEffector && ChainStart->AttachedEffector->RotateLimb)
		{
			ChainStartRot = RotateStartToEnd * ChainStart->AttachedEffector->Rotation;
		}
		else
		{
			ChainStartRot = RotateStartToEnd * ChainStart->Rotation;
		}
		if (ChainEnd->AttachedEffector && ChainEnd->AttachedEffector->RotateLimb)
		{
			ChainEndRot = RotateStartToEnd * ChainEnd->AttachedEffector->Rotation;
		}
		else
		{
			ChainEndRot = RotateStartToEnd * ChainEnd->Rotation;
		}

		// calculate axis to squash along
		Vec3 EndToStart = ChainStart->Position - ChainEnd->Position;
		float ChainLength = std::max<float>(0.1f, EndToStart.Length());
		float SquashPercent = 1.0f - (ChainLength / Chain.LineLengthOrig);

		for (unsigned int k = 1, f = 0; k < Chain.Bones.size() - 1; ++k, ++f)
		{
			Bone* Bone = Chain.Bones[k];

			// do not squash solved fork children (they are rigid)
			if (Bone->IsChildOfSolvedFork)
			{
				continue;
			}

			// calculate blended intermediate position
			const Vec3& LocalPositionStart = Chain.BonesRelativeToStart[f];
			const Vec3& LocalPositionEnd = Chain.BonesRelativeToEnd[f];
			const float& WeightToStart = Chain.BoneParams[f];
			Vec3 StartPosition = ChainStart->Position + ChainStartRot.Rotate(LocalPositionStart);
			Vec3 EndPosition = ChainEnd->Position + ChainEndRot.Rotate(LocalPositionEnd);
			Vec3 BlendedBonePosition = Vec3::Lerp(EndPosition, StartPosition, WeightToStart);

			// are we squashing or stretching?
			if (ChainLength >= Chain.LineLengthOrig)
			{
				// stretching: blended position is good enough
				Bone->Position = BlendedBonePosition;
			}
			else
			{
				// squashing
				Vec3 PointOnLine = ChainEnd->Position + EndToStart * WeightToStart;

				// determine direction to squash bone
				Vec3 SquashDir;
				if (Bone->UseBendDirection)
				{
					// squash in direction of custom bend vector (knees, elbows etc)
					SquashDir = (RotateStartToEnd * Bone->Rotation).Rotate(Bone->BendDirection);
					// TODO still not sure which is better
					//SquashDir = Bone->Rotation.Rotate(Bone->BendDirection);
				}
				else
				{
					// squash perpendicular to chain axis
					SquashDir = BlendedBonePosition - PointOnLine;
				}

				SquashDir.NormalizeSafe();
				Vec3 SquashOffset = (SquashDir * (Bone->DistToParent * 5.0f)) * SquashPercent;
				Bone->Position = BlendedBonePosition + SquashOffset;
			}
		}
	}
}

void Core::RotateSubChainRoots()
{
	//UpdateSolvedBoneRotations();

	// rotate free-forks (non affected) towards new effector positions
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& Fork = Bones[i];
		if (Fork.AttachedEffector || !Fork.IsSolvedFork)
		{
			continue;
		}

		// get chain to orient
		SubChain* Chain = Fork.ChildSubChain;
		float Weight = 1.f / (float)Chain->NeighborChains.size();

		// apply root rotation multiplier
		if (Fork.IsSolverRoot)
		{
			Weight *= MainSettings.RootRotationMultiplier;
		}

		// accumulate rotations to point to neighboring chains
		for (unsigned int j = 0; j < Chain->NeighborChains.size(); ++j)
		{
			SubChain* NeighborChain = Chain->NeighborChains[j];
			Bone* NeighborRoot = NeighborChain->Bones[0];
			Vec3 ToNeighborOrig = (NeighborRoot->PositionFromPose - Fork.PositionFromPose).Normalized();
			Vec3 ToNeighborNew = (NeighborRoot->Position - Fork.Position).Normalized();
			Quat ForkRotation = Quat::FromToRotation(ToNeighborOrig, ToNeighborNew);
			ForkRotation = Quat::Lerp(Quat::Identity(), ForkRotation, Weight);
			Fork.Rotation = (ForkRotation * Fork.Rotation).Normalize();
		}

		// TODO - apply stiffness to rotation of free forks?

		// update children positions after new rotation (locked relative to fork)
		SetChildrenToLocalPositionsFromPose(&Fork);
	}
}

/*
void Core::ApplyStiffness()
{
	UpdateSolvedBoneRotations();

	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		if (!(B.IsSolved && B.Stiffness >= 0.001f))
		{
			continue;
		}

		// solver root doesn't have parent to be stiff relative to,
		// so we blend it back to it's input position.
		if (B.IsSolverRoot)
		{
			B.Position = Vec3::Lerp(B.Position, B.PositionFromPose, B.Stiffness);
			B.Rotation = Quat::Lerp(B.Rotation, B.RotationFromPose, B.Stiffness);
			continue;
		}

		// calculate current local position of bone
		Vec3 LocalPosition = B.Parent->Rotation.Inverse().Rotate(B.Position - B.Parent->Position);
		Vec3 StiffLocalPosition = Vec3::Lerp(LocalPosition, B.LocalPositionFromPose, B.Stiffness);
		B.Position = B.Parent->Position + B.Parent->Rotation.Rotate(StiffLocalPosition);
	}
}*/

void Core::SolveSubRootPullConstraints()
{
	// reset accumulated position corrections
	for (int i = 0; i < (int)SubChains.size(); ++i)
	{
		SubChains[i].FABRIKPositions.clear();
	}

	// BACKWARD (LEAVES to ROOT) 
	// iterate through each depth of sub-chains from LEAVES to ROOT
	for (int d = (int)SubChains.size() - 1; d >= 0; --d)
	{
		SubChain& Chain = SubChains[d];

		// apply accumulated position corrections to tip,
		// including any effector position changes
		MoveSubChainTipBackPass(&Chain);

		// no parent chain to pull?
		if (Chain.Bones.size() <= 1)
		{
			continue;
		}
		
		// move root closer to tip if necessary, but do not push subroots!
		// subroots are only pulled, not pushed. squashing of subroots is allowed
		// and managed by the chain squashing behavior
		Bone* ChainRoot = Chain.Bones[Chain.Bones.size() - 1];
		Bone* ChainTip = Chain.Bones[0];

		float DistToRoot;
		Vec3 RootToTipNorm;
		Vec3 RootToTip = ChainTip->Position - Chain.SquashStartBone->Position;
		RootToTip.GetDirectionAndLength(RootToTipNorm, DistToRoot);
		Vec3 NewPosition;
		if (DistToRoot > Chain.ChainLength)
		{
			Vec3 Correction = RootToTipNorm * (DistToRoot - Chain.ChainLength);

			// factor in PullWeight of tip of chain IF it has an effector
			if (ChainTip->AttachedEffector)
			{
				float PullWeight = ChainTip->AttachedEffector->PullWeight;
				PullWeight = std::max(0.0f, std::min(1.0f, PullWeight));

				Correction = ScaleDeltaByEffectorPullWeights(Correction, PullWeight, *ChainTip->AttachedEffector);
			}

			NewPosition = ChainRoot->Position + Correction;
		}else
		{
			NewPosition = ChainRoot->Position;
		}

		ChainRoot->ChildSubChain->FABRIKPositions.push_back(NewPosition);
	}

	// FORWARD (ROOT to LEAVES)
	// iterate through each depth of sub-chains from ROOT to LEAVES
	for (int d = 0; d < (int)SubChains.size(); ++d)
	{
		SubChain& Chain = SubChains[d];

		// no child chain to pull?
		if (Chain.Bones.size() <= 1)
		{
			continue;
		}

		// pull tip closer to root if necessary, but do not push subroots!
		Bone* ChainTip = Chain.Bones[0];
		float DistToTip;
		Vec3 TipToRootNorm;
		Vec3 TipToRoot = Chain.SquashStartBone->Position - ChainTip->Position;
		TipToRoot.GetDirectionAndLength(TipToRootNorm, DistToTip);
		Vec3 NewPosition;
		if (DistToTip > Chain.ChainLength)
		{
			Vec3 Correction = TipToRootNorm * (DistToTip - Chain.ChainLength);
			ChainTip->Position += Correction;
			if (ChainTip->IsSolvedFork)
			{
				SetChildrenToLocalPositionsFromPose(ChainTip);
			}
		}
	}
}

void Core::SolveSubChainDistanceConstraints()
{
	float DistanceConstraintStrength = 0.9f;

	// iterate over all sub-chains and adjust lengths of bones
	for (int i = (int)SubChains.size() - 1; i >= 0; --i)
	{
		SubChain& Chain = SubChains[i];
		if (Chain.Bones.size() <= 2)
		{
			continue; // no intermediate Bones to solve
		}

		for (int k = (int)Chain.Bones.size() - 2; k >= 0; --k)
		{
			Bone* B = Chain.Bones[k];
			Bone* Parent = Chain.Bones[k + 1];
			if (!Parent)
				continue; // root
			Vec3 AToB = Parent->Position - B->Position;
			Vec3 AToBNorm = AToB.NormalizedSafe();
			Vec3 Delta = (AToB - (AToBNorm * B->DistToParent)) * 0.5f;
			Delta *= DistanceConstraintStrength;
			if (!(B->AttachedEffector || B->IsSolvedFork || B->IsChildOfSolvedFork))
				B->Position += Delta;
			if (!(Parent->AttachedEffector || Parent->IsSolvedFork || Parent->IsChildOfSolvedFork))
				Parent->Position -= Delta;
		}
	}
}

void Core::SolveSubChainBendingConstraints(const int& iteration)
{
	const float BendingConstraintStrength = 1.0f;
	float Stiffness = BendingConstraintStrength > 0.001f ? BendingConstraintStrength : 0.001f;

	// linear stiffness independent of iterations
	// PAGE 5: http://matthias-mueller-fischer.ch/publications/posBasedDyn.pdf
	const float IterFloat = static_cast<float>(iteration);
	Stiffness = 1.0f - powf((1.0f - Stiffness), (1.0f / (IterFloat + 1.0f)));

	// iterate over all sub-chains and adjust lengths of bones
	for (int I = (int)SubChains.size() - 1; I >= 0; --I)
	{
		SubChain& SubChain = SubChains[I];
		if (SubChain.Bones.size() <= 2)
		{
			continue; // no intermediate Bones to squash
		}

		for (int K = 0; K < (int)SubChain.BendingConstraints.size() - 1; ++K)
		{
			// back/forth iteration
			//k = iteration % 2 == 0 ? k : SubChain.BendingConstraints.size() - k - 1;

			// triangle bending constraint 
			// FIGURE 6: https://pdfs.semanticscholar.org/759f/e17efeb59ab2081135db3d1517093bbcb085.pdf 
			BendConstraint& BendConstraint = SubChain.BendingConstraints[K];
			Vec3 CenterToB = BendConstraint.CentroidToB();
			const float CorrMag = 1.0f - (BendConstraint.CentroidToBOrig / CenterToB.Length());
			const Vec3 Delta = CenterToB * (CorrMag * Stiffness);
			const Vec3 HalfDelta = Delta * 0.5f;
			if (!BendConstraint.AIsLocked)
				BendConstraint.A->Position += HalfDelta; // 2/4
			if (!BendConstraint.CIsLocked)
				BendConstraint.C->Position += HalfDelta; // 2/4
			if (!BendConstraint.BIsLocked)
				BendConstraint.B->Position -= Delta;  // 4/4
		}
	}
}

void Core::SolveFABRIKBackward()
{
	//
	// BACKWARD (leaves to root)
	//

	// reset accumulated position corrections
	for (int i = 0; i < (int)SubChains.size(); ++i)
	{
		SubChains[i].FABRIKPositions.clear();
	}

	// iterate through each depth of sub-chains from leaf to root
	for (int d = (int)SubChains.size() - 1; d >= 0; --d)
	{
		SubChain& Chain = SubChains[d];
		MoveSubChainTipBackPass(&Chain);
		Bone* Child = Chain.Bones[0];
		Effector* TipEftr = Child->AttachedEffector;

		// go BACKWARDS (child to parent) along the chain and adjust each bone position
		for (int b = 1; b < (int)Chain.Bones.size() - 1; ++b)
		{
			// move parent along vector from self to parent
			Bone* Parent = Chain.Bones[b];
			Vec3 TargetPosition = Child->Position + (Parent->Position - Child->Position).NormalizedSafe() * Child->DistToParent;
			// move the bone, taking into consideration constraints
			MoveBoneDuringBackwardPass(Parent, TargetPosition, TipEftr);
			Child = Parent;
		}
	}
}

void Core::SolveFABRIKForward()
{
	//
	// FORWARD (root to leaves)
	//

	// iterate through each depth of sub-chains from root to leaf
	for (int d = 0; d < (int)SubChains.size(); ++d)
	{
		SubChain& Chain = SubChains[d];

		// go FORWARD (parent to child) along the chain and adjust each bone position
		Bone* Parent = Chain.Bones[Chain.Bones.size() - 1];
		for (int b = (int)Chain.Bones.size() - 2; b >= 0; --b)
		{
			// calculate new target position for bone
			Bone* Child = Chain.Bones[b];
			Vec3 TargetPosition = Parent->Position + (Child->Position - Parent->Position).NormalizedSafe() * Child->DistToParent;
			// move the bone, taking into consideration constraints
			MoveBoneDuringForwardPass(Child, TargetPosition);
			Parent = Child;
		}
	}
}

void Core::MoveSubChainTipBackPass(SubChain* Chain)
{
	//
	// during BACKWARD pass from LEAVES to ROOT
	//
	Bone* TipBone = Chain->Bones[0];

	// in case where sub-chain has an attached effector, 
	// apply the effector's position and rotation directly
	if (TipBone->AttachedEffector)
	{
		// NOTE: we used to just set the TipBone to the effector position
		// but it's better to average the effector influence with the rest of the body
		// this is especially important for intermediate effectors not on the tip of the chain
		Chain->FABRIKPositions.push_back(TipBone->AttachedEffector->Position);
		if (TipBone->AttachedEffector->RotateBone)
		{
			TipBone->Rotation = TipBone->AttachedEffector->Rotation;
		}
	}
	
	// apply average position
	if (Chain->FABRIKPositions.size() > 0)
	{
		Vec3 PositionSum = Vec3::Zero();
		for (int i = 0; i < (int)Chain->FABRIKPositions.size(); ++i)
			PositionSum += Chain->FABRIKPositions[i];
		TipBone->Position = PositionSum * (1.0f / (float)Chain->FABRIKPositions.size());
		Chain->FABRIKPositions.clear();
	}

	// rigidly move all children of the fork, 
	// they are not allowed to move relative to the fork
	// we need updated fork-child positions for subsequent iterations
	if (TipBone->IsSolvedFork)
	{
		SetChildrenToLocalPositionsFromPose(TipBone);
	}
}

void Core::MoveSubChainRootFwdPass(SubChain* Chain)
{
	//
	// during FORWARD pass from ROOT to LEAVES
	//

	Bone* RootBone = Chain->Bones[Chain->Bones.size() - 1];

	if (RootBone->AttachedEffector)
	{
		RootBone->Position = RootBone->AttachedEffector->Position;
	}

	if (RootBone->IsSolvedFork)
	{
		Core::SetChildrenToLocalPositionsFromPose(RootBone);
	}
}

void Core::MoveBoneDuringForwardPass(Bone* B, Vec3 TargetPosition)
{
	// we are moving a Bone to target position during the FORWARD pass
	// when the algorithm goes from ROOT to TIP...

	// only pull forks when inertia NOT being applied (will override inertia)
	if (B->IsChildOfSolvedFork && Inertia.ApplyInertiaToBody)
	{
		return;
	}

	if (B->IsChildOfSolvedFork)
	{
		Vec3 Delta = (TargetPosition - B->Position);
		B->Parent->Position += Delta;
		Core::SetChildrenToLocalPositionsFromPose(B->Parent);
	}
	else
	{
		B->Position = TargetPosition;
	}

	if (B->IsSolvedFork)
	{
		SetChildrenToLocalPositionsFromPose(B);
	}
}

void Core::MoveBoneDuringBackwardPass(Bone* B, Vec3 TargetPosition, Effector* TipEftr)
{
	// we are moving a Bone to target position during the BACKWARD pass
	// when the algorithm goes from TIP to ROOT...

	// only pull forks when inertia NOT being applied (will override inertia)
	if (B->IsChildOfSolvedFork && Inertia.ApplyInertiaToBody)
	{
		return;
	}

	// scale target position if TipEftr is pulling a fork
	if (TipEftr && (B->ChildSubChain || B->IsChildOfSolvedFork))
	{
		// clamp PullWeight between 0-1
		float PullWeight = std::min(1.0f, TipEftr->PullWeight);
		PullWeight = std::max(0.0f, PullWeight);

		// scale linearly by pull weight first
		Vec3 Delta = (TargetPosition - B->Position) * PullWeight;

		// factor in non-linear directional weights
		Delta.X *= Delta.X > 0.0f ? TipEftr->PositivePullFactor.X : TipEftr->NegativePullFactor.X;
		Delta.Y *= Delta.Y > 0.0f ? TipEftr->PositivePullFactor.Y : TipEftr->NegativePullFactor.Y;
		Delta.Z *= Delta.Z > 0.0f ? TipEftr->PositivePullFactor.Z : TipEftr->NegativePullFactor.Z;

		TargetPosition = B->Position + Delta;
	}

	// this bone is the child of a solved fork 
	//
	// apply it's translation correction to the fork, AND all the fork's children
	if (B->IsChildOfSolvedFork)
	{
		// calc delta to move bone to target
		Vec3 Delta = TargetPosition - B->Position;
		Bone* Fork = B->Parent;
		Vec3 NewForkPosition = Fork->Position + Delta;
		Fork->ChildSubChain->FABRIKPositions.push_back(NewForkPosition);
		// update child too
		B->Position = TargetPosition;
		return;
	}

	// just a regular intermediate bone
	if (B->Stiffness > 0.001f)
	{
		// SHOULD PROBABLY DO THIS IN LOCAL SPACE
		B->Position = Vec3::Lerp(TargetPosition, B->Position, B->Stiffness);
	}
	else
	{
		B->Position = TargetPosition;
	}
}

void Core::UpdateNonSolvedSkeletonBelowBone(Bone& Start)
{
	for (int i = 0; i < (int)Start.AllChildren.size(); ++i)
	{
		Bone* Child = Start.AllChildren[i];

		if (Child->IsSolved)
		{
			continue;
		}

		if (!Child->Parent)
		{
			continue;
		}

		Bone* Parent = Child->Parent;
		Child->Position = Parent->Position + Parent->Rotation.Rotate(Child->LocalPositionFromPose);
		Child->Rotation = Parent->Rotation * Child->LocalRotationFromPose;
	}
}

void Core::UpdateSolvedBoneRotations()
{
	for (int i = 0; i < (int)Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		if (!B.IsSolved)
		{
			continue;
		}

		// rotate to effector if it has one
		if (B.AttachedEffector && B.AttachedEffector->RotateBone)
		{
			B.Rotation = B.AttachedEffector->Rotation;
			continue;
		}

		// tip joints don't get rotated by the solver,
		// just restore local rotation relative to parent
		if (B.Children.size() <= 0)
		{
			B.Rotation = B.Parent->Rotation * B.LocalRotationFromPose;
			continue;
		}

		// solved fork orientations are handled by prior function
		if (Bones[i].IsSolvedFork)
		{
			continue;
		}

		// update normal bone by rotating it towards it's first solved child
		for (int j = 0; j < (int)B.Children.size(); ++j)
		{
			Bone& Child = *B.Children[j];
			if (!Child.IsSolved)
			{
				continue;
			}

			Vec3 OrigDir = Child.PositionFromPose - B.PositionFromPose;
			Vec3 NewDir = Child.Position - B.Position;
			// rotate from old pose to new one
			Quat Rotation = Quat::FromToRotation(OrigDir, NewDir);
			B.Rotation = Rotation * B.RotationFromPose;
			break;
		}
	}
}

void Core::UpdatePoleVectorRotations()
{
	for (int i = (int)SubChains.size() - 1; i >= 0; --i)
	{
		SubChain& Chain = SubChains[i];
		if (Chain.Bones.size() <= 2)
		{
			continue; // no intermediate Bones
		}

		Bone* TipBone = Chain.Bones[0];
		Effector *Eftr = TipBone->AttachedEffector;
		if (!Eftr)
		{
			continue; // no effector, then can't have pole vector
		}

		const EffectorPoleVector& PoleVector = Eftr->PoleVector;
		if (PoleVector.Mode == PoleVectorMode::NONE)
		{
			continue; // no pole vector on this effector
		}

		// calculate pole axis of chain
		Bone* StartBone = Chain.SquashStartBone;
		Vec3 Start = StartBone->Position;
		Vec3 End = TipBone->Position;
		Vec3 Axis = (Start - End).NormalizedSafe();

		// determine rotation for chain joints
		Quat PoleRotation;
			
		// determine angle to rotate around axis
		if (PoleVector.Mode == PoleVectorMode::ANGLE_OFFSET)
		{
			PoleRotation = Quat::FromAxisAngleDegrees(Axis, PoleVector.AngleOffset);
		}
		else
		{
			// get the target position of the pole vector
			Vec3 PoleTarget;
			if (PoleVector.Mode == PoleVectorMode::BONE)
			{
				PoleTarget = Bones[PoleVector.BoneIndex].Position;
			}
			else
			{
				PoleTarget = PoleVector.Position;
			}
				
			// get the initial position of the pole vector
			Vec3 TargetOrig = Eftr->PoleTargetRelativeToStartOrig;
			TargetOrig = Start + StartBone->RotationFromPose.Rotate(TargetOrig);
			// project it in to the limb plane
			Vec3 TargetOrigInPolePlane = Vec3::ProjectPointOnPlane(TargetOrig, End, Axis);
			TargetOrigInPolePlane = (TargetOrigInPolePlane - End).NormalizedSafe();

			// get the target position of the pole vector in the limb plane
			Vec3 TargetNewInPolePlane = Vec3::ProjectPointOnPlane(PoleTarget, End, Axis);
			TargetNewInPolePlane = (TargetNewInPolePlane - End).NormalizedSafe();

			// calc rotation between them
			PoleRotation = Quat::FromToRotation(TargetOrigInPolePlane, TargetNewInPolePlane);
		}
			
		// rotate the chain bones by the pole rotation
		for (int b = 1; b < (int)Chain.Bones.size() - 1; ++b)
		{
			Bone* ChainBone = Chain.Bones[b];
			Vec3 BoneVec = ChainBone->Position - End;
			ChainBone->Position = End + PoleRotation.Rotate(BoneVec);
			ChainBone->Rotation = PoleRotation * ChainBone->Rotation;
		}
	}
}

bool Core::GetBoneExists(const char* BoneName)
{
	for (int i = 0; i < (int)Bones.size(); ++i)
	{
		if (Bones[i].Name == BoneName)
		{
			return true;
		}
	}
	return false;
}

bool Core::AreBonesConnected(Bone* A, Bone* B)
{
	// return false if the path between A and B is blocked by an effector
	if (A == B)
		return true;

	std::vector<Bone*> AToUnblockedRootBones;
	Bone* ParentOfA = A;
	while (ParentOfA)
	{
		// case where B in direct path to root
		if (ParentOfA == B)
			return true;

		// effectors BETWEEN A and B block the path
		if (ParentOfA != A && ParentOfA->AttachedEffector != nullptr)
			break;

		// store bone
		AToUnblockedRootBones.push_back(ParentOfA);

		ParentOfA = ParentOfA->Parent;
	}

	// find lowest common ancestor
	Bone* ParentOfB = B;
	while (ParentOfB)
	{
		// check if this is a common ancestor of A
		for (int i = 0; i < (int)AToUnblockedRootBones.size(); ++i)
		{
			if (AToUnblockedRootBones[i] == ParentOfB)
				return true;
		}

		// effectors BETWEEN A and B block the path
		if (ParentOfB != B && ParentOfB->AttachedEffector != nullptr)
			return false;

		ParentOfB = ParentOfB->Parent;
	}

	return false;
}

void Core::SetChildrenToLocalPositionsFromPose(Bone* Fork)
{
	for (unsigned int i = 0; i < Fork->Children.size(); ++i)
	{
		Bone* Child = Fork->Children[i];
		Child->Position = Fork->Position + Fork->Rotation.Rotate(Child->LocalPositionFromPose);
	}
}

float Core::DistanceBetweenBones(Bone* A, Bone* B)
{
	if (A == B)
	{
		return 0.0f;
	}

	std::vector<Bone*> AToRootBones;
	std::vector<float> AToRootDistances;
	Bone* ParentOfA = A;
	float DistanceFromA = 0.0f;
	while (ParentOfA)
	{
		AToRootBones.push_back(ParentOfA);

		DistanceFromA += ParentOfA->DistToParent;
		AToRootDistances.push_back(DistanceFromA);

		// case where B in direct path to root
		if (ParentOfA == B)
			return DistanceFromA;

		ParentOfA = ParentOfA->Parent;
	}

	// find lowest common ancestor
	Bone* ParentOfB = B;
	float DistanceFromB = 0.0f;
	while (ParentOfB)
	{
		DistanceFromB += ParentOfB->DistToParent;

		// check if this is a common ancestor of A
		for (int i = 0; i < (int)AToRootBones.size(); ++i)
		{
			if (AToRootBones[i] == ParentOfB)
				return DistanceFromB + AToRootDistances[i];
		}
		ParentOfB = ParentOfB->Parent;
	}

	// could not find distance between Bones in Skeleton. This should not happen.
	// (TODO - prove I don't need to add error reporting mechanism for this case)
	return -1.0f;
}


} // namespace
