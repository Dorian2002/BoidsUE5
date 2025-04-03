 // Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Containers/Array.h"
#include "Math/MathFwd.h"
#include "CollisionShape.h"
#include "BoidsSystem.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(Boids, Log, All);
class UInstancedStaticMeshComponent;

UCLASS()
class BOIDS_API ABoidsSystem : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ABoidsSystem();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	// Boid data arrays for position, velocity, and acceleration
	TArray<FVector3d> Positions;
	TArray<FVector3d> Velocities;
	TArray<FVector3d> Accelerations;
	TArray<FRotator3d> Rotations;
	
	// Prepare new transform array for batch update
	TArray<FTransform> NewTransforms;
	
	uint32 CurrentBoidsCount;
	
	//Offset rotation so the mesh is well oriented
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	FRotator RotationOffset = FRotator(-90,0,0);
	
	UPROPERTY()
	UInstancedStaticMeshComponent* InstanceMesh;

	//Use sphere trace instead of line traces
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	bool bUseSphereCollision = false;
	FCollisionShape CollisionSphere;
	
	// Avoidance directions used for collision
	TArray<FVector3d> AvoidanceDirections;

	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	uint32 NumBoids = 1000;

	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	UStaticMesh* BoidMesh;

	// Radius for each behavior (separation, alignment, cohesion, avoidance)
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double SeparationRadius = 80.0;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double AlignmentRadius = 200.0;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double CohesionRadius = 300.0;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double AvoidanceRadius = 150.0;

	// Squared Radius for calculus (less precise but faster)
	double SeparationRadiusSq;
	double AlignmentRadiusSq;
	double CohesionRadiusSq;
	double AvoidanceRadiusSq;
	
	// Weights to scale the effects of each behavior
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double SeparationWeight = 1.1;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double AlignmentWeight = 1.0;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double CohesionWeight = 1.3;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double AvoidanceWeight = 2.0;

	// Other configurable parameters for boids movements and vision
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double MaxSpeed = 250.0;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double MaxSteeringForce = 50.0;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	double FOVAngle = 120.0;
	// Pre compute a cosines calculus that will always be the same at runtime
	float PreComputeCos;
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	uint32 AvoidanceRayCount = 16;  // Number of rays to cast for avoidance

	// Fixed update
	UPROPERTY(EditAnywhere, Category="Boids Configuration")
	float FixedDeltaTime = 1.0f / 30.0f;  // 30Hz physics update
	float TimeAccumulator = 0.0f;

	// Methods for initialization and updating of boid states
	void InitializeBoids();
	void UpdateBoids(float DeltaTime);

	// Utility functions
	TArray<FVector3d> GetAvoidanceDirections(uint32 NumPoints) const;

	void ComputeBoidForces(
		uint32 BoidIndex,
		FVector3d& OutSeparation, 
		FVector3d& OutAlignment, 
		FVector3d& OutCohesion);
	
	FVector3d ComputeCollisionAvoidance(uint32 BoidIndex);
	FVector3d ComputeLineTraceCollisionForce(uint32 BoidIndex);
	FVector3d ComputeSphereTraceCollisionForce(uint32 BoidIndex);

	void ExtrapolateBoids(float Alpha);

	// Function to check if another boid is in the Field of View
	bool IsInFOV(const FVector3d& Forward, const FVector3d& ToOther) const;
};
