/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#include "PowerIKCore.h"

#include <queue>
#include <map>


namespace PowerIK
{

using PowerIK::Core;
using PowerIK::Bone;
using PowerIK::Effector;
using PowerIK::BendConstraint;
using PowerIK::SubChain;
using PowerIK::Quat;
using PowerIK::Vec3;

void Core::Initialize()
{
	// At this stage, the Bones and Effectors are in a state where:
	// 1. they have starting positions / rotations
	// 2. they have names of their parents (but parent pointers are null)
	//
	// This is enough to pre-calculate the missing meta data the solver requires:
	// - pointers to parents
	// - children and all children arrays are NULL
	// - effector hierarchy is NULL
	// - orig pos/rot are zero
	// - initial bone lengths
	// - etc...
	InitBones();
	InitEffectors();
	InitSubChains();
	InitSmoothWeights();
	InitBendingConstraints();
	InitPoleVectors();
	IsInitialized = true;
}

void Core::InitBones()
{
	// init Bone pointers to parents
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		if (B.ParentIndex >= 0)
		{
			B.Parent = &Bones[B.ParentIndex];
		}
	}

	// re-parent around excluded bones
	// effectively removing them from the simulation
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];

		// find first non excluded parent
		Bone* Parent = B.Parent;
		while (Parent)
		{
			if (!Parent->IsExcluded)
			{
				B.Parent = Parent;
				break;
			}

			Parent = Parent->Parent;
		}
	}

	// init Bone Children array
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		for (unsigned int j = 0; j < Bones.size(); ++j)
		{
			Bone& Child = Bones[j];
			if (Child.Parent == &B)
			{
				B.Children.push_back(&Child);
			}
		}
	}

	// init Bone AllChildren array
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		
		std::queue<Bone*> ChildrenToAdd;
		for (unsigned int j = 0; j <B.Children.size(); ++j)
		{
			ChildrenToAdd.push(B.Children[j]);
		}
			
		while (!ChildrenToAdd.empty())
		{
			Bone* NextChild = ChildrenToAdd.front();
			ChildrenToAdd.pop();
			B.AllChildren.push_back(NextChild);
			for (int j = 0; j < (int)NextChild->Children.size(); ++j)
			{
				ChildrenToAdd.push(NextChild->Children[j]);
			}
		}
	}

	// init Bone lengths
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		if (!B.Parent)
		{
			continue;
		}
			
		B.DistToParent = (B.Position - B.Parent->Position).Length();
	}
}

void Core::InitEffectors()
{
	// init pointers from Effectors to Bones
	for (unsigned int i = 0; i < Effectors.size(); ++i)
	{
		Effector& Eftr = Effectors[i];
		for (unsigned int j = 0; j < Bones.size(); ++j)
		{
			if (Eftr.BoneName == Bones[j].Name)
			{
				Eftr.Bone = &Bones[j];
				break;
			}
		}
	}

	// init IsSolved bool on Bones
	// init AttachedEffector pointers on Bones
	// 
	// "Solved" bones are located between effectors and the solver root (inclusive)
	// "Non-Solved" bones are simply kinematic children
	for (unsigned int i = 0; i < Effectors.size(); ++i)
	{
		Effector& Eftr = Effectors[i];
		Eftr.Bone->IsSolved = true;
		Eftr.Bone->AttachedEffector = &Eftr;

		// search upwards until we hit solver root and set IsSolved to true
		Bone* Parent = Eftr.Bone->Parent;
		while (Parent)
		{
			Parent->IsSolved = true;
			if (Parent->IsSolverRoot)
				break;
			Parent = Parent->Parent;
		}
	}

	// init IsSolvedFork and IsChildOfSolvedFork bools on Bones
	// (solved forks have multiple SOLVED children bones)
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		if (B.Children.size() <= 1)
		{
			continue; // can't be a fork
		}

		int NumSolvedChildren = 0;
		for (unsigned int j = 0; j < B.Children.size(); ++j)
			NumSolvedChildren += 1 ? B.Children[j]->IsSolved : 0;

		if (NumSolvedChildren > 1)
		{
			B.IsSolvedFork = true;
			// init IsChildOfSolvedFork state
			for (unsigned int j = 0; j < B.Children.size(); ++j)
				B.Children[j]->IsChildOfSolvedFork = true;
		}
	}

	// initialize prev position/rotation for temporal smoothing
	for (unsigned int i = 0; i < Effectors.size(); ++i)
	{
		Effector& Eftr = Effectors[i];
		Eftr.PrevPosition = Eftr.Position;
		Eftr.PrevRotation = Eftr.Rotation;
	}
}

void Core::InitSubChains()
{
	// create list of all SubChains, recording:
	// 1. their depth (from root)
	// 2. their list of Bones in their chain
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];

		// SubRoots are Solved bones:
		// 1. with an attached effector OR
		// 2. that fork to multiple children OR
		// 3. the actual root of the solved skeleton
		bool IsSubRoot = B.IsSolved && (B.IsSolvedFork || B.IsSolverRoot || B.AttachedEffector);
		if (!IsSubRoot)
		{
			continue;
		}

		SubChain Chain;
		Chain.Bones.push_back(&B);
		Chain.Depth = 0;
		Bone* Parent = !B.IsSolverRoot ? B.Parent : nullptr;
		bool PassedEndOfChain = false;
		while (Parent)
		{
			if (!PassedEndOfChain)
			{
				Chain.Bones.push_back(Parent);
			}

			if (Parent->AttachedEffector || Parent->IsSolvedFork || Parent->IsSolverRoot)
			{
				Chain.Depth += 1;
				PassedEndOfChain = true;
			}

			if (Parent->IsSolverRoot)
			{
				break;
			}

			Parent = Parent->Parent;
		}

		// init prev position for inertia
		//Chain.Velocity = Vec3::Zero();
		Chain.Position = Chain.Bones[Chain.Bones.size()-1]->Position;
		Chain.Velocity = Vec3::Zero();
		//Chain.PrevPosition = Chain.Position;

		SubChains.push_back(Chain);
	}

	// find start of squashing for each SubChain
	for (unsigned int j = 0; j < SubChains.size(); ++j)
	{
		SubChain& Chain = SubChains[j];
		Chain.SquashStartBone = Chain.Bones[Chain.Bones.size() - 1];

		// SquashStartBone may be different than the root of the Subchain 
		// if the root is a solved fork we don't want to calculate the squash 
		// from the fork, but rather from the fork's CHILD. the fork itself can 
		// be far from the root of the actual limb in most creatures and will 
		// skew the squashing direction of otherwise straight limbs
		if (Chain.SquashStartBone->IsSolvedFork && (int)Chain.Bones.size() > 2)
			Chain.SquashStartBone = Chain.Bones[Chain.Bones.size() - 2];
	}

	// connect Bones to SubChains
	for (unsigned int i = 0; i < SubChains.size(); ++i)
	{
		SubChain& Chain = SubChains[i];
		Chain.Bones[0]->ChildSubChain = &Chain;
	}

	// connect SubChains to neighbors
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bone& B = Bones[i];
		if (B.IsSolverRoot || !B.ChildSubChain)
		{
			continue;
		}

		Bone* Parent = B.Parent;
		while (Parent)
		{
			if (Parent->ChildSubChain)
			{
				Parent->ChildSubChain->NeighborChains.push_back(B.ChildSubChain);
				B.ChildSubChain->NeighborChains.push_back(Parent->ChildSubChain);
				break;
			}

			Parent = Parent->Parent;
		}
	}

	// calculate length of chain
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

void Core::InitSmoothWeights()
{
	// calculate max distance between solved bones for normalizing
	float MaxDistanceBetweenBones = 0.0f;
	int NumCalcs = 0;
	std::map<std::pair<int, int>, bool> Distances; // memoized to avoid n^2
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		if (!Bones[i].IsSolved)
		{
			continue;
		}

		for (unsigned int j = 0; j < Bones.size(); ++j)
		{
			if (!Bones[j].IsSolved)
			{
				continue;
			}

			std::pair<int, int> BonePair;
			BonePair.first = i <= j ? i : j;
			BonePair.second = i > j ? i : j;
			if (!(Distances.count(BonePair) > 0))
			{
				float Distance = DistanceBetweenBones(&Bones[i], &Bones[j]);
				if (Distance > MaxDistanceBetweenBones)
					MaxDistanceBetweenBones = Distance;
				Distances[BonePair] = true;
				NumCalcs++;
			}
		}
	}

	// increase max distance so that if there is a sub root that is exactly max distance
	// from an effector it will still be weighted to the effector because it's InvDistance
	// will still be a positive value greater than zero.
	MaxDistanceBetweenBones *= 1.5f;

	// calculate subroot effector weights
	for (unsigned int i = 0; i < SubChains.size(); ++i)
	{
		SubChain& Sub = SubChains[i];

		// calculate distance to all effectors
		for (unsigned int j = 0; j < Effectors.size(); ++j)
		{
			Effector& Eftr = Effectors[j];
			float DistanceAlongChain = DistanceBetweenBones(Sub.Bones[0], Eftr.Bone);
			float InvDistance = MaxDistanceBetweenBones - DistanceAlongChain;
			InvDistance *= InvDistance; // squared for pleasing falloff
			Sub.EffectorDistances.push_back(InvDistance);
			Sub.EffectorDistWeights.push_back(InvDistance);
			Sub.EffectorPullWeights.push_back(InvDistance);
		}
	}

	// normalize the weights
	NormalizeEffectorWeights();
}

void Core::InitBendingConstraints()
{
	// create bending constraints for each sub chain
	for (unsigned int i = 0; i < SubChains.size(); ++i)
	{
		SubChain& Chain = SubChains[i];
		if (Chain.Bones.size() <= 2)
		{
			continue; // no intermediate bones in sub chain
		}

		// iterate from second to second-last bone, because bending constraints
		// are between three bones: the middle bone and it's parent/child
		for (unsigned int k = 1; k < Chain.Bones.size() - 2; ++k)
		{
			BendConstraint BendConstraint;
			BendConstraint.A = Chain.Bones[k - 1];
			BendConstraint.B = Chain.Bones[k];
			BendConstraint.C = Chain.Bones[(unsigned int)(k + 1)];
			BendConstraint.Initialize();
			Chain.BendingConstraints.push_back(BendConstraint);
		}
	}
}

void Core::InitPoleVectors()
{
	for (unsigned int i = 0; i < SubChains.size(); ++i)
	{
		const SubChain& Chain = SubChains[i];
			
		if (!Chain.Bones[0]->AttachedEffector)
		{
			continue; // no effector on this sub chain
		}

		if (Chain.Bones.size() <= 2)
		{
			continue; // no intermediate bones in sub chain
		}

		Effector& Eftr = *Chain.Bones[0]->AttachedEffector;
		if (Eftr.PoleVector.Mode == PoleVectorMode::NONE
			|| Eftr.PoleVector.Mode == PoleVectorMode::ANGLE_OFFSET)
		{
			continue; // neither of these modes cache a pole vector
		}

		// get the initial target position of the pole vector
		Vec3 PoleTarget;
		if (Eftr.PoleVector.Mode == PoleVectorMode::BONE)
		{
			PoleTarget = Bones[Eftr.PoleVector.BoneIndex].Position;
		}
		else
		{
			PoleTarget = Eftr.PoleVector.Position;
		}

		/*
		// project target position into chain axis
		Bone* ChainEnd = Chain.Bones[0];
		Bone* ChainStart = Chain.SquashStartBone;
		Vec3 Start = ChainStart->Position;
		Vec3 End = ChainEnd->Position;
		Vec3 Axis = (End - Start).NormalizedSafe();
		PoleTarget = Vec3::Project(PoleTarget, Axis).NormalizedSafe();
		*/

		// store pole target position relative to start bone
		PoleTarget = PoleTarget - Chain.SquashStartBone->Position;
		Eftr.PoleTargetRelativeToStartOrig = Chain.SquashStartBone->Rotation.Inverse().Rotate(PoleTarget);
	}
}

} //namespace