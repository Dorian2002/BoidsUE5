// Fill out your copyright notice in the Description page of Project Settings.

#include "BoidsSystem.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Kismet/KismetMathLibrary.h"


DEFINE_LOG_CATEGORY(Boids);

// Sets default values
ABoidsSystem::ABoidsSystem()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Create Instanced Static Mesh Component for boids visualization
	InstanceMesh = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstanceMesh"));
	InstanceMesh->SetCollisionResponseToChannel(ECC_WorldStatic, ECollisionResponse::ECR_Ignore);
	RootComponent = InstanceMesh;
}

// Called when the game starts or when spawned
void ABoidsSystem::BeginPlay()
{
	Super::BeginPlay();

	PreComputeCos = FMath::Cos(FMath::DegreesToRadians(FOVAngle * 0.5));

	NewTransforms.SetNumUninitialized(NumBoids);
	Rotations.SetNumUninitialized(NumBoids);
	Positions.SetNumUninitialized(NumBoids);
	Velocities.SetNumUninitialized(NumBoids);
	Accelerations.SetNumUninitialized(NumBoids);

	CollisionSphere = FCollisionShape::MakeSphere(AvoidanceRadius);
	
	SeparationRadiusSq = SeparationRadius*SeparationRadius;
	AlignmentRadiusSq = AlignmentRadius*AlignmentRadius;
	CohesionRadiusSq = CohesionRadius*CohesionRadius;
	AvoidanceRadiusSq = AvoidanceRadius*AvoidanceRadius;
	
	// Calls boids Initialization
	InitializeBoids();
}

// Called every frame
void ABoidsSystem::Tick(float DeltaTime)
{
	TimeAccumulator += DeltaTime;
    
	if(TimeAccumulator >= FixedDeltaTime)
	{
		// Perform the fixed update step
		UpdateBoids(FixedDeltaTime);

		TimeAccumulator -= FixedDeltaTime;
	}
	else
	{
		// Interpolate for rendering
		const float Alpha = TimeAccumulator / FixedDeltaTime;
		UE_LOG(Boids, Log, TEXT("%f"),Alpha)
		ExtrapolateBoids(Alpha);
	}
}

// Sets initial positions, velocities, and meshes for boids
void ABoidsSystem::InitializeBoids()
{
	// Assign a Static Mesh and Material
	if (BoidMesh)
		InstanceMesh->SetStaticMesh(BoidMesh);
	
	const FVector3d NewAcceleration = FVector3d(0,0,0);
	
	// Initialize Boid Data and create instances for each boid
	for (uint32 i = 0; i < NumBoids; i++)
	{
		FVector3d NewPosition = FVector3d(FMath::RandRange(-3000, 3000), FMath::RandRange(-3000, 3000), FMath::RandRange(0, 3000));
		const FVector3d RandomDirection = UKismetMathLibrary::RandomUnitVector()*MaxSpeed;
		FVector3d NewVelocity = RandomDirection;

		Positions[i] = NewPosition;
		Velocities[i] = NewVelocity;
		Accelerations[i] = NewAcceleration;

		CurrentBoidsCount++;
		InstanceMesh->AddInstance(FTransform(NewPosition));
	}

	if (!bUseSphereCollision)
		//Only used for line traces
		AvoidanceDirections = GetAvoidanceDirections(AvoidanceRayCount);
}

// Updates boids positions and velocities based on behaviors
void ABoidsSystem::UpdateBoids(float DeltaTime)
{
    if (CurrentBoidsCount == 0) return;

    // Parallel update of boid positions (thread safe as we're not moving/deleting boids from arrays)
    ParallelFor(CurrentBoidsCount, [&](uint32 i)
    {
        // Compute steering forces using the optimized spatial methods
        FVector3d Alignment;
        FVector3d Cohesion;
        FVector3d Separation;
        const FVector3d Avoidance = ComputeCollisionAvoidance(i);
    	ComputeBoidForces(i, Separation, Alignment, Cohesion);

        // Apply initial steering from alignment and cohesion
        FVector3d SteeringForce = Alignment + Cohesion;

        if (!Separation.IsNearlyZero())
        {
            SteeringForce = Separation; // Override with separation if too close
        }

        SteeringForce += Avoidance;

        // Update velocity and position, clamped to max speed
        const FVector3d Velocity = Velocities[i];
        Velocities[i] = (Velocity + SteeringForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
        Positions[i] += Velocities[i];

    	Rotations[i] = Velocity.ToOrientationRotator() + RotationOffset;

        // Update instance transform for rendering
        NewTransforms[i] = FTransform(Rotations[i], Positions[i]);
    });

    // Batch update all instances to set the new positions
    InstanceMesh->BatchUpdateInstancesTransforms(0, NewTransforms, true, true, true);
}

//Generates directions for collision avoidance rays
TArray<FVector3d> ABoidsSystem::GetAvoidanceDirections(uint32 NumPoints) const
{
	TArray<FVector3d> Directions;
	const double GoldenRatio = (1.0 + FMath::Sqrt(5.0)) / 2.0;

	for (uint32 i = 0; i < NumPoints; i++)
	{
		// Golden ratio for even distribution of rays
		const double Theta = FMath::Acos(1 - 2 * (i + 0.5) / NumPoints);
		const double Phi = 2.0 * PI * GoldenRatio * i;

		FVector3d Direction;
		Direction.X = FMath::Sin(Theta) * FMath::Cos(Phi);
		Direction.Y = FMath::Sin(Theta) * FMath::Sin(Phi);
		Direction.Z = FMath::Cos(Theta);

		Directions.Add(Direction);
	}

	return Directions;
}

void ABoidsSystem::ComputeBoidForces(
	uint32 BoidIndex,
	FVector3d& OutSeparation, 
	FVector3d& OutAlignment, 
	FVector3d& OutCohesion)
{
	FVector3d SeparationForce = FVector3d::ZeroVector;
	FVector3d AlignmentForce = FVector3d::ZeroVector;
	FVector3d CohesionPositionSum = FVector3d::ZeroVector;

	uint32 SeparationCount = 0;
	uint32 AlignmentCount = 0;
	uint32 CohesionCount = 0;

	const FVector3d& BoidPos = Positions[BoidIndex];

	for (uint32 j = 0; j < NumBoids; j++)
	{
		if (j == BoidIndex) continue;
		FVector3d ToOther = Positions[j] - BoidPos;
		
		FVector3d ToOtherNormalized = ToOther;
		ToOtherNormalized.Normalize();
		
		if (!IsInFOV(Velocities[BoidIndex], ToOtherNormalized))
			continue;
		const double DistSq = ToOther.SizeSquared();
		
		// Separation
		if (DistSq < SeparationRadiusSq && DistSq > 1e-4)
		{
			SeparationForce += -ToOtherNormalized / FMath::InvSqrt(DistSq);
			SeparationCount++;
		}
		
		// Alignment
		if (DistSq < AlignmentRadiusSq)
		{
			AlignmentForce += Velocities[j];
			AlignmentCount++;
		}
		
		// Cohesion
		if (DistSq < CohesionRadiusSq)
		{
			CohesionPositionSum += Positions[j];
			CohesionCount++;
		}
	}

	// Final weighted forces
	OutSeparation = (SeparationCount > 0) ? SeparationForce.GetSafeNormal() * SeparationWeight : FVector3d::ZeroVector;
	OutAlignment = (AlignmentCount > 0) ? (AlignmentForce / AlignmentCount).GetSafeNormal() * AlignmentWeight : FVector3d::ZeroVector;
	OutCohesion = (CohesionCount > 0) ? ((CohesionPositionSum / CohesionCount - BoidPos).GetSafeNormal() * CohesionWeight) : FVector3d::ZeroVector;
}

//Calculates collision avoidance using raycasting
FVector3d ABoidsSystem::ComputeCollisionAvoidance(uint32 BoidIndex)
{	
	if (bUseSphereCollision)
		return ComputeSphereTraceCollisionForce(BoidIndex);
	return ComputeLineTraceCollisionForce(BoidIndex);
}

FVector3d ABoidsSystem::ComputeLineTraceCollisionForce(uint32 BoidIndex)
{
	double BestDistanceSq = AvoidanceRadiusSq;
	FVector3d AvoidanceForce = FVector3d::ZeroVector;
	// Uses rays to check for obstacles in front of the boid and applies avoidance force
	for (const FVector3d& Dir : AvoidanceDirections)
	{
		FVector3d TraceStart = Positions[BoidIndex];
		FVector3d RayEnd = TraceStart + (Dir * AvoidanceRadius);
		
		if (FHitResult Hit; GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, RayEnd, ECC_WorldStatic))
		{
			const double DistanceSq = (Hit.ImpactPoint - TraceStart).SizeSquared();
			if (DistanceSq < BestDistanceSq)
			{
				BestDistanceSq = DistanceSq;

				// Steer in the opposite direction of the hit normal (push away from the wall)
				AvoidanceForce = Hit.Normal * AvoidanceWeight;
			}
		}
	}

	return AvoidanceForce;  // Apply as an avoidance force;
}

FVector3d ABoidsSystem::ComputeSphereTraceCollisionForce(uint32 BoidIndex)
{
	double BestDistanceSq = AvoidanceRadiusSq;
	FVector3d AvoidanceForce = FVector3d::ZeroVector;
	FVector3d TraceStart = Positions[BoidIndex];
	TArray<FHitResult> HitActors;
	
	GetWorld()->SweepMultiByChannel(
		HitActors, TraceStart, TraceStart, FQuat::Identity,
		ECC_WorldStatic, CollisionSphere
	);

	for (FHitResult Hit : HitActors)
	{
		const double DistanceSq = (Hit.ImpactPoint - TraceStart).SizeSquared();
		if (DistanceSq < BestDistanceSq)
		{
			BestDistanceSq = DistanceSq;

			// Steer in the opposite direction of the hit normal (push away from the wall)
			AvoidanceForce = Hit.Normal * AvoidanceWeight;
		}
	}

	return AvoidanceForce;  // Apply as an avoidance force;
}

void ABoidsSystem::ExtrapolateBoids(float Alpha)
{
	ParallelFor(CurrentBoidsCount, [&](uint32 i)
   {
		// Update instance transform for rendering based on previous tick velocity
		NewTransforms[i] = FTransform(Rotations[i], Positions[i] + Velocities[i] * Alpha * FixedDeltaTime);
   });

	// Batch update all instances to set the new positions
	InstanceMesh->BatchUpdateInstancesTransforms(0, NewTransforms, true, true, true);
}

bool ABoidsSystem::IsInFOV(const FVector3d& Forward, const FVector3d& ToOther) const
{
	// Uses dot product to check if a target is in the FOV of the boid
	return FVector3d::DotProduct(Forward, ToOther) > PreComputeCos;
}
