// file path: carla\Unreal\CarlaUE4\Plugins\Carla\Source\Carla\Vehicle

// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "WheeledVehicleAIController.h"

#include "MapGen/RoadMap.h"
#include "Traffic/RoutePlanner.h"
#include "Vehicle/CarlaWheeledVehicle.h"

#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "WheeledVehicleMovementComponent.h"

#include <math.h>

#define CLAMP(__v, __hi, __lo) (__v > __hi? __hi : (__v < __lo? __lo: __v))

namespace PIDControllerConstants {

  const float VELOCITY_INTEGRAL_MAX = 5.0f;
  const float VELOCITY_INTEGRAL_MIN = -5.0f;

} // namespace PIDControllerConstants

using namespace PIDControllerConstants;

// =============================================================================
// -- Static local methods -----------------------------------------------------
// =============================================================================

static bool RayCast(const AActor &Actor, const FVector &Start, const FVector &End)
{
  FHitResult OutHit;
  static FName TraceTag = FName(TEXT("VehicleTrace"));
  FCollisionQueryParams CollisionParams(TraceTag, true);
  CollisionParams.AddIgnoredActor(&Actor);

  const bool Success = Actor.GetWorld()->LineTraceSingleByObjectType(
      OutHit,
      Start,
      End,
      FCollisionObjectQueryParams(FCollisionObjectQueryParams::AllDynamicObjects),
      CollisionParams);

  // DrawDebugLine(Actor.GetWorld(), Start, End,
  //     Success ? FColor(255, 0, 0) : FColor(0, 255, 0), false);

  return Success && OutHit.bBlockingHit;
}

static bool IsThereAnObstacleAhead(
    const ACarlaWheeledVehicle &Vehicle,
    const float Speed,
    const FVector &Direction)
{
  const auto ForwardVector = Vehicle.GetVehicleOrientation();
  const auto VehicleBounds = Vehicle.GetVehicleBoundingBoxExtent();

  FVector NormDirection = Direction.GetSafeNormal();

  const float Distance = std::max(50.0f, Speed * Speed); // why?

  const FVector StartCenter = Vehicle.GetActorLocation() +
      (ForwardVector * (250.0f + VehicleBounds.X / 2.0f)) + FVector(0.0f, 0.0f, 50.0f);
  const FVector EndCenter = StartCenter + NormDirection * (Distance + VehicleBounds.X / 2.0f);

  const FVector StartRight = StartCenter +
      (FVector(ForwardVector.Y, -ForwardVector.X, ForwardVector.Z) * 100.0f);
  const FVector EndRight = StartRight + NormDirection * (Distance + VehicleBounds.X / 2.0f);

  const FVector StartLeft = StartCenter +
      (FVector(-ForwardVector.Y, ForwardVector.X, ForwardVector.Z) * 100.0f);
  const FVector EndLeft = StartLeft + NormDirection * (Distance + VehicleBounds.X / 2.0f);

  return
    RayCast(Vehicle, StartCenter, EndCenter) ||
    RayCast(Vehicle, StartRight, EndRight) ||
    RayCast(Vehicle, StartLeft, EndLeft);
}

// function to detect the preceding vehicle (if any) for the ACC system
static bool DetectVehicleAhead(const ACarlaWheeledVehicle &Vehicle, FHitResult &ClosestVehicle)
{
  float Distance = 100 * 100.0f;
  float HitRadius = 0.5 * 100.0f;
  bool bOnlyDynamics = true;

  const FVector &Start = Vehicle.GetActorLocation();
  const FVector &End = Start + (Vehicle.GetActorForwardVector() * Distance);
  UWorld* CurrentWorld = Vehicle.GetWorld();

  // Struct in which the result of the scan will be saved
  FHitResult HitOut = FHitResult();

  // Initialization of Query Parameters
  FCollisionQueryParams TraceParams(FName(TEXT("PrecedingVehicleDetection Trace")), true, &Vehicle);

  // Hit against complex meshes
  TraceParams.bTraceComplex = true;

  // Ignore trigger boxes
  TraceParams.bIgnoreTouches = true;

  // Limit the returned information
  TraceParams.bReturnPhysicalMaterial = false;

  // Ignore ourselves
  TraceParams.AddIgnoredActor(&Vehicle);

  bool isHitReturned;
  // Choosing a type of sweep is a workaround until everything get properly
  // organized under correct collision channels and object types.
  // If we go only for dynamics, we check the object type AllDynamicObjects
  FCollisionObjectQueryParams TraceChannel = FCollisionObjectQueryParams(
      FCollisionObjectQueryParams::AllDynamicObjects);
  isHitReturned = CurrentWorld->SweepSingleByObjectType(
      HitOut,
      Start,
      End,
      FQuat(),
      TraceChannel,
      FCollisionShape::MakeSphere(HitRadius),
      TraceParams);

  if (isHitReturned)
  {
    // check if the hit object is a vehicle
    if (ATagger::MatchComponent(*HitOut.Component, ECityObjectLabel::Vehicles)) {
      ClosestVehicle = HitOut;
      return true;
    }
  }
  return false;
}

// calculate the target speed that the PID controller wants to achieve
// according to "On a Formal Model of Safe and Scalable Self-driving Cars" Shai Shalev-Shwartz, Shaked Shammah, Amnon Shashua
// vehicle A is in front of vehicle B
float AWheeledVehicleAIController::CalculateTargetSpeed(const float Speed, FHitResult &ClosestVehicle, const float DeltaTime, bool &accControl)
{
  float max_brake = 9.8f; //m/s^2
  float max_accel = 5.4f; //m/s^2
  float min_brake = 2.9f; //m/s^2
  float Throttle;

  AController *VehicleAController = (ClosestVehicle.Actor)->GetInstigatorController();
  AWheeledVehicleAIController *VehicleAVehControl = dynamic_cast<AWheeledVehicleAIController*> ( VehicleAController );
  if(VehicleAVehControl != nullptr)
  {
    ACarlaWheeledVehicle *VehicleA = VehicleAVehControl->GetPossessedVehicle();
    float VelocityA = VehicleA->GetVehicleForwardSpeed() / 100.0f ; // m/s
    float VelocityB = Speed / 3.6f; // m/s
    float VelocityRel = VelocityB - VelocityA;
    // if not modified 1 unit (returned by ClosestVehicle.Distance) is 1 cm
    float currentDist = ClosestVehicle.Distance / 100.0f; // meters
    // calculate safety distance
    float safetyDist = VelocityB*0.05 + max_accel * 0.00125 -pow(VelocityA, 2) / (2*max_brake) + pow((VelocityB+ 0.05 * max_accel), 2)/ (2*min_brake);
    if (safetyDist < 3)
    {
      safetyDist = 3; //minimum distance between two vehicles
    }
    float distAvailable = currentDist - safetyDist;
    float timeAvailable = distAvailable / (VelocityRel);
    if(distAvailable < 0) // current distance is less than the safety distance
    {
      // the host vehicle needs to stop
      Throttle = -1.0f;
      accControl = false;
    }
    else if(currentDist >= 2 * safetyDist || (timeAvailable > 5 && VelocityB > 10)) // the front vehicle is too far
    {
      // the host vehicle moves normally
      Throttle = Move(Speed);
      accControl = false;
    }
    else
    {
      if (VelocityA < 0.01f && VelocityB < 0.01f && distAvailable < 2)
      {
        return -1.0f;
      }
      float Acceleration;
      if (VelocityRel < 0) // the front car is moving faster
      {
        float desiredSpeed = (VelocityA * 1.1 > SpeedLimit) ? SpeedLimit : (VelocityA*1.1);
        Acceleration = (desiredSpeed - VelocityB) / 5;
      }
      else // the host car is moving faster
      {
        Acceleration = (VelocityA - VelocityB) / timeAvailable;
      }
      float TargetSpeed = Acceleration * DeltaTime + VelocityB;
      if (TargetSpeed > SpeedLimit/3.6f)
      {
        TargetSpeed = SpeedLimit;
      }
      // calculate the throttle with the pid controller
      Throttle = run_step(VelocityB*3.6f, TargetSpeed*3.6f, DeltaTime);
    }

    return Throttle;
    }
    else
    {
    UE_LOG(LogCarla, Error, TEXT("Preceding Vehicle Detected but its controller is not WheeledVehicleAIController!"));
    return 1;
    }
}

// PID controller
float AWheeledVehicleAIController::run_step (const float current_speed, const float target_speed, const float DeltaTime)
{
  float e = (target_speed - current_speed) / target_speed;
  float dt = DeltaTime;
  float de, ie;

  ie = dt * e + velocity_intergal;
  ie = CLAMP(ie, VELOCITY_INTEGRAL_MAX, VELOCITY_INTEGRAL_MIN);

  de = (e - prev_velocity) / dt;

  float output = args_longitudinal.K_P * e
                  + args_longitudinal.K_D * de
                  + args_longitudinal.K_I * ie ;

  //update values
  velocity_intergal = ie;
  prev_velocity = e;

  if (output < -1)
  {
      output = -1;
  }
  else if (output > 1)
  {
      output = 1;
  }
  return output; // return throttle
}

template <typename T>
static void ClearQueue(std::queue<T> &Queue)
{
  std::queue<T> EmptyQueue;
  Queue.swap(EmptyQueue);
}

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

AWheeledVehicleAIController::AWheeledVehicleAIController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));

  RandomEngine->Seed(RandomEngine->GenerateRandomSeed());

  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;
}

AWheeledVehicleAIController::~AWheeledVehicleAIController() {}

// =============================================================================
// -- AController --------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::OnPossess(APawn *aPawn)
{
  Super::OnPossess(aPawn);

  if (IsPossessingAVehicle())
  {
    UE_LOG(LogCarla, Error, TEXT("Controller already possessing a vehicle!"));
    return;
  }
  Vehicle = Cast<ACarlaWheeledVehicle>(aPawn);
  check(Vehicle != nullptr);
  MaximumSteerAngle = Vehicle->GetMaximumSteerAngle();
  check(MaximumSteerAngle > 0.0f);
  ConfigureAutopilot(bAutopilotEnabled);

  if (RoadMap == nullptr)
  {
    TActorIterator<ACityMapGenerator> It(GetWorld());
    RoadMap = (It ? It->GetRoadMap() : nullptr);
  }
}

void AWheeledVehicleAIController::OnUnPossess()
{
  Super::OnUnPossess();

  Vehicle = nullptr;
}

void AWheeledVehicleAIController::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);

  if (!IsPossessingAVehicle())
  {
    return;
  }

  if (bAutopilotEnabled)
  {
    Vehicle->ApplyVehicleControl(TickAutopilotController(DeltaTime), EVehicleInputPriority::Autopilot);
  }
  else if (!bControlIsSticky)
  {
    Vehicle->ApplyVehicleControl(FVehicleControl{}, EVehicleInputPriority::Relaxation);
  }

  Vehicle->FlushVehicleControl();
}

// =============================================================================
// -- Autopilot ----------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::ConfigureAutopilot(const bool Enable, const bool KeepState)
{
  bAutopilotEnabled = Enable;
  if (!KeepState)
  {
    // Reset state.
    Vehicle->SetSteeringInput(0.0f);
    Vehicle->SetThrottleInput(0.0f);
    Vehicle->SetBrakeInput(0.0f);
    Vehicle->SetReverse(false);
    Vehicle->SetHandbrakeInput(false);
    ClearQueue(TargetLocations);
    Vehicle->SetAIVehicleState(
        bAutopilotEnabled ?
        ECarlaWheeledVehicleState::FreeDriving :
        ECarlaWheeledVehicleState::AutopilotOff);
  }

  TrafficLightState = ETrafficLightState::Green;

  /// @todo Workaround for a race condition between client and server when
  /// enabling autopilot right after initializing a vehicle.
  if (bAutopilotEnabled)
  {
    for (TActorIterator<ARoutePlanner> It(GetWorld()); It; ++It)
    {
      ARoutePlanner *RoutePlanner = *It;
      // Check if we are inside this route planner.
      TSet<AActor *> OverlappingActors;
      RoutePlanner->TriggerVolume->GetOverlappingActors(
          OverlappingActors,
          ACarlaWheeledVehicle::StaticClass());
      for (auto *Actor : OverlappingActors)
      {
        if (Actor == Vehicle)
        {
          RoutePlanner->AssignRandomRoute(*this);
          return;
        }
      }
    }
  }
}

// =============================================================================
// -- ACC ----------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::ConfigureACC(const bool Enable)
{
  bACCEnabled = Enable;
  args_longitudinal = {.K_P = 1.0f, .K_D = 0.0005f, .K_I = 1.0f};
  //args_longitudinal = {.K_P = 2.0f, .K_D = 0.4f, .K_I = 0.01f};
}

// =============================================================================
// -- Traffic ------------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::SetFixedRoute(
    const TArray<FVector> &Locations,
    const bool bOverwriteCurrent)
{
  if (bOverwriteCurrent)
  {
    ClearQueue(TargetLocations);
  }
  for (auto &Location : Locations)
  {
    TargetLocations.emplace(Location);
  }
}

// =============================================================================
// -- AI -----------------------------------------------------------------------
// =============================================================================

FVehicleControl AWheeledVehicleAIController::TickAutopilotController(const float DeltaTime)
{
#if WITH_EDITOR // This happens in simulation mode in editor.
  if (Vehicle == nullptr)
  {
    bAutopilotEnabled = false;
    return {};
  }
#endif // WITH_EDITOR

  check(Vehicle != nullptr);

  FVector Direction;

  float Steering;
  if (!TargetLocations.empty())
  {
    Steering = GoToNextTargetLocation(Direction);
  }
  else
  {
    Steering = RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    Direction = Vehicle->GetVehicleTransform().GetRotation().Rotator().Vector();
  }

  const float speed_cm_s = Vehicle->GetVehicleForwardSpeed();
  // Speed in km/h.
  const auto Speed = speed_cm_s * 0.036f;

  FHitResult ClosestVehicle;

  float Throttle;
  bool accControl = false;
  if (TrafficLightState != ETrafficLightState::Green)
  {
    Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::WaitingForRedLight);
    Throttle = Stop(Speed);
  }
  else if (bACCEnabled && DetectVehicleAhead(*Vehicle, ClosestVehicle))
  {
    accControl = true;
    Throttle = CalculateTargetSpeed(Speed, ClosestVehicle, DeltaTime, accControl);
  }
  else if (IsThereAnObstacleAhead(*Vehicle, Speed, Direction))
  {
    Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::ObstacleAhead);
    Throttle = Stop(Speed);
  }
  else
  {
    Throttle = Move(Speed);
  }

  FVehicleControl AutopilotControl;

  if (accControl)
  {
    if (Throttle < 0)
    {
      AutopilotControl.Brake = abs(Throttle);
      AutopilotControl.Throttle = 0.0f;
    }
    else
    {
      AutopilotControl.Brake = 0.0f;
      AutopilotControl.Throttle = Throttle;
    }
  }
  else
  {
    if (Throttle < 0.001f)
    {
      AutopilotControl.Brake = 1.0f;
      AutopilotControl.Throttle = 0.0f;
    }
    else
    {
      AutopilotControl.Brake = 0.0f;
      AutopilotControl.Throttle = Throttle;
    }
  }
  AutopilotControl.Steer = Steering;

  return AutopilotControl;
}

float AWheeledVehicleAIController::GoToNextTargetLocation(FVector &Direction)
{
  // Get middle point between the two front wheels.
  const auto CurrentLocation = [&]() {
    const auto &Wheels = Vehicle->GetVehicleMovementComponent()->Wheels;
    check((Wheels.Num() > 1) && (Wheels[0u] != nullptr) && (Wheels[1u] != nullptr));
    return (Wheels[0u]->Location + Wheels[1u]->Location) / 2.0f;
  } ();

  const auto Target = [&]() {
    const auto &Result = TargetLocations.front();
    return FVector{Result.X, Result.Y, CurrentLocation.Z};
  } ();

  if (Target.Equals(CurrentLocation, 200.0f))
  {
    TargetLocations.pop();
    if (!TargetLocations.empty())
    {
      return GoToNextTargetLocation(Direction);
    }
    else
    {
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
  }

  Direction = (Target - CurrentLocation).GetSafeNormal();

  const FVector &Forward = GetPawn()->GetActorForwardVector();

  float dirAngle = Direction.UnitCartesianToSpherical().Y;
  float actorAngle = Forward.UnitCartesianToSpherical().Y;

  dirAngle *= (180.0f / PI);
  actorAngle *= (180.0 / PI);

  float angle = dirAngle - actorAngle;

  if (angle > 180.0f)
  {
    angle -= 360.0f;
  }
  else if (angle < -180.0f)
  {
    angle += 360.0f;
  }

  float Steering = 0.0f;
  if (angle < -MaximumSteerAngle)
  {
    Steering = -1.0f;
  }
  else if (angle > MaximumSteerAngle)
  {
    Steering = 1.0f;
  }
  else
  {
    Steering += angle / MaximumSteerAngle;
  }

  Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::FollowingFixedRoute);
  return Steering;
}

float AWheeledVehicleAIController::CalcStreeringValue(FVector &direction)
{
  float steering = 0;
  FVector BoxExtent = Vehicle->GetVehicleBoundingBoxExtent();
  FVector forward = Vehicle->GetActorForwardVector();

  FVector rightSensorPosition(BoxExtent.X / 2.0f, (BoxExtent.Y / 2.0f) + 100.0f, 0.0f);
  FVector leftSensorPosition(BoxExtent.X / 2.0f, -(BoxExtent.Y / 2.0f) - 100.0f, 0.0f);

  float forwardMagnitude = BoxExtent.X / 2.0f;

  float Magnitude =
      (float) sqrt(pow((double) leftSensorPosition.X, 2.0) + pow((double) leftSensorPosition.Y, 2.0));

  // same for the right and left
  float offset = FGenericPlatformMath::Acos(forwardMagnitude / Magnitude);

  float actorAngle = forward.UnitCartesianToSpherical().Y;

  float sinR = FGenericPlatformMath::Sin(actorAngle + offset);
  float cosR = FGenericPlatformMath::Cos(actorAngle + offset);

  float sinL = FGenericPlatformMath::Sin(actorAngle - offset);
  float cosL = FGenericPlatformMath::Cos(actorAngle - offset);

  rightSensorPosition.Y = sinR * Magnitude;
  rightSensorPosition.X = cosR * Magnitude;

  leftSensorPosition.Y = sinL * Magnitude;
  leftSensorPosition.X = cosL * Magnitude;

  FVector rightPositon = GetPawn()->GetActorLocation() + FVector(rightSensorPosition.X,
      rightSensorPosition.Y,
      0.0f);
  FVector leftPosition = GetPawn()->GetActorLocation() + FVector(leftSensorPosition.X,
      leftSensorPosition.Y,
      0.0f);

  FRoadMapPixelData rightRoadData = RoadMap->GetDataAt(rightPositon);
  if (!rightRoadData.IsRoad())
  {
    steering -= 0.2f;
  }

  FRoadMapPixelData leftRoadData = RoadMap->GetDataAt(leftPosition);
  if (!leftRoadData.IsRoad())
  {
    steering += 0.2f;
  }

  FRoadMapPixelData roadData = RoadMap->GetDataAt(GetPawn()->GetActorLocation());
  if (!roadData.IsRoad())
  {
    steering = 0.0f;
  }
  else if (roadData.HasDirection())
  {

    direction = roadData.GetDirection();
    FVector right = rightRoadData.GetDirection();
    FVector left = leftRoadData.GetDirection();

    forward.Z = 0.0f;

    float dirAngle = direction.UnitCartesianToSpherical().Y;
    float rightAngle = right.UnitCartesianToSpherical().Y;
    float leftAngle = left.UnitCartesianToSpherical().Y;

    dirAngle *= (180.0f / PI);
    rightAngle *= (180.0 / PI);
    leftAngle *= (180.0 / PI);
    actorAngle *= (180.0 / PI);

    float min = dirAngle - 90.0f;
    if (min < -180.0f)
    {
      min = 180.0f + (min + 180.0f);
    }

    float max = dirAngle + 90.0f;
    if (max > 180.0f)
    {
      max = -180.0f + (max - 180.0f);
    }

    if (dirAngle < -90.0 || dirAngle > 90.0)
    {
      if (rightAngle < min && rightAngle > max)
      {
        steering -= 0.2f;
      }
      if (leftAngle < min && leftAngle > max)
      {
        steering += 0.2f;
      }
    }
    else
    {
      if (rightAngle < min || rightAngle > max)
      {
        steering -= 0.2f;
      }
      if (leftAngle < min || leftAngle > max)
      {
        steering += 0.2f;
      }
    }

    float angle = dirAngle - actorAngle;

    if (angle > 180.0f)
    {
      angle -= 360.0f;
    }
    else if (angle < -180.0f)
    {
      angle += 360.0f;
    }

    if (angle < -MaximumSteerAngle)
    {
      steering = -1.0f;
    }
    else if (angle > MaximumSteerAngle)
    {
      steering = 1.0f;
    }
    else
    {
      steering += angle / MaximumSteerAngle;
    }
  }

  Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::FreeDriving);
  return steering;
}

float AWheeledVehicleAIController::Stop(const float Speed)
{
  return (Speed >= 1.0f ? -Speed / SpeedLimit : 0.0f);
}

float AWheeledVehicleAIController::Move(const float Speed)
{
  if (Speed >= SpeedLimit)
  {
    return Stop(Speed);
  }
  else if (Speed >= SpeedLimit - 10.0f)
  {
    return 0.5f;
  }
  else
  {
    return 1.0f;
  }
}
