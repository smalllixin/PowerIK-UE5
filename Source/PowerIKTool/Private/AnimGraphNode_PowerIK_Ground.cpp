/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#include "AnimGraphNode_PowerIK_Ground.h"
#include "Components/SkeletalMeshComponent.h"
#include "PowerIKRuntime/Public/AnimNode_PowerIK_Ground.h"
#include "Animation/AnimInstance.h"

#define LOCTEXT_NAMESPACE "A3Nodes"

UAnimGraphNode_PowerIK_Ground::UAnimGraphNode_PowerIK_Ground(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_PowerIK_Ground::GetControllerDescription() const
{
	return LOCTEXT("PowerIKGround", "Power IK Ground");
}

FText UAnimGraphNode_PowerIK_Ground::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_PowerIK_Tooltip", "PowerIK's ground alignment node.");
}

FText UAnimGraphNode_PowerIK_Ground::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

void UAnimGraphNode_PowerIK_Ground::Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* SkelMeshComp) const
{
	if (bEnableDebugDraw && SkelMeshComp)
	{
		UAnimInstance* AnimInstance = SkelMeshComp->GetAnimInstance();
		UObject* AnimInstanceUObject = Cast<UObject>(AnimInstance);
		FAnimNode_PowerIK_Ground* ActiveNode = GetActiveInstanceNode<FAnimNode_PowerIK_Ground>(AnimInstanceUObject);
		if (ActiveNode)
		{
			ActiveNode->ConditionalDebugDraw(PDI, SkelMeshComp);
		}
	}
}

#undef LOCTEXT_NAMESPACE
