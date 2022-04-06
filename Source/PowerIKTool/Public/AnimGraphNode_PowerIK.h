/*
Copyright 2020 Power Animated, All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "PowerIKRuntime/Public/AnimNode_PowerIK.h"
#include "AnimGraphNode_PowerIK.generated.h"

UCLASS(MinimalAPI)
class UAnimGraphNode_PowerIK : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_UCLASS_BODY()

	UPROPERTY(EditAnywhere, Category = Settings)
	FAnimNode_PowerIK Node;

	/** Enable drawing of debug information in viewport. */
	UPROPERTY(EditAnywhere, Category = Debug)
	bool bEnableDebugDraw = true;

public:
	// UEdGraphNode interface
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	// End of UEdGraphNode interface

protected:

	// UAnimGraphNode_SkeletalControlBase interface
	virtual void Draw(FPrimitiveDrawInterface* PDI, USkeletalMeshComponent* SkelMeshComp) const override;
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }
	// End of UAnimGraphNode_SkeletalControlBase interface
};
