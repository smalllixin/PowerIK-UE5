/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#include "AnimGraphNode_PowerIK_Walk.h"
#include "Components/SkeletalMeshComponent.h"
#include "PowerIKRuntime/Public/AnimNode_PowerIK_Walk.h"
#include "Animation/AnimInstance.h"

#define LOCTEXT_NAMESPACE "A3Nodes"

UAnimGraphNode_PowerIK_Walk::UAnimGraphNode_PowerIK_Walk(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_PowerIK_Walk::GetControllerDescription() const
{
	return LOCTEXT("PowerIKWalk", "Power IK Walk");
}

FText UAnimGraphNode_PowerIK_Walk::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_PowerIK_Walk_Tooltip", "PowerIK's procedural locomotion node.");
}

FText UAnimGraphNode_PowerIK_Walk::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

void UAnimGraphNode_PowerIK_Walk::Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* SkelMeshComp) const
{
	if (bEnableDebugDraw && SkelMeshComp)
	{
		UAnimInstance* AnimInstance = SkelMeshComp->GetAnimInstance();
		UObject* AnimInstanceUObject = Cast<UObject>(AnimInstance);
		FAnimNode_PowerIK_Walk* ActiveNode = GetActiveInstanceNode<FAnimNode_PowerIK_Walk>(AnimInstanceUObject);
		if (ActiveNode)
		{
			ActiveNode->ConditionalDebugDraw(PDI, SkelMeshComp);
		}
	}
}

#undef LOCTEXT_NAMESPACE
