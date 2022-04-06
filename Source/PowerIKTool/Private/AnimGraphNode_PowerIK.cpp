/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#include "AnimGraphNode_PowerIK.h"
#include "Components/SkeletalMeshComponent.h"
#include "PowerIKRuntime/Public/AnimNode_PowerIK.h"
#include "Animation/AnimInstance.h"

#define LOCTEXT_NAMESPACE "A3Nodes"

UAnimGraphNode_PowerIK::UAnimGraphNode_PowerIK(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_PowerIK::GetControllerDescription() const
{
	return LOCTEXT("PowerIK", "Power IK Solver");
}

FText UAnimGraphNode_PowerIK::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_PowerIK_Tooltip", "PowerIK's full-body IK solver.");
}

FText UAnimGraphNode_PowerIK::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

void UAnimGraphNode_PowerIK::Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* SkelMeshComp) const
{
	if (bEnableDebugDraw && SkelMeshComp)
	{
		UAnimInstance* AnimInstance = SkelMeshComp->GetAnimInstance();
		UObject* AnimInstanceUObject = Cast<UObject>(AnimInstance);
		FAnimNode_PowerIK* ActiveNode = GetActiveInstanceNode<FAnimNode_PowerIK>(AnimInstanceUObject);
		if (ActiveNode)
		{
			ActiveNode->ConditionalDebugDraw(PDI, SkelMeshComp);
		}
	}
}

#undef LOCTEXT_NAMESPACE
