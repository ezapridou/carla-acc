// file path: carla\Unreal\CarlaUE4\Plugins\Carla\Source\Carla\Vehicle

// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <queue>
#include <iostream>
#include <deque>

#include "GameFramework/Controller.h"

#include "Traffic/TrafficLightState.h"
#include "Vehicle/VehicleControl.h"

#include "WheeledVehicleAIController.generated.h"

class ACarlaWheeledVehicle;
class URandomEngine;
class URoadMap;

struct args_long {
  float K_P;
  float K_D;
  float K_I;
} ;

/// Wheeled vehicle controller with optional AI.
UCLASS()
class CARLA_API AWheeledVehicleAIController final : public AController
{
  GENERATED_BODY()

  // ===========================================================================
  /// @name Constructor and destructor
  // ===========================================================================
  /// @{

public:

  AWheeledVehicleAIController(const FObjectInitializer &ObjectInitializer);

  ~AWheeledVehicleAIController();

  /// @}
  // ===========================================================================
  /// @name Controller overrides
  // ===========================================================================
  /// @{

public:

  void OnPossess(APawn *aPawn) override;

  void OnUnPossess() override;

  void Tick(float DeltaTime) override;

  /// @}
  // ===========================================================================
  /// @name Possessed vehicle
  // ===========================================================================
  /// @{

public:

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  bool IsPossessingAVehicle() const
  {
    return Vehicle != nullptr;
  }

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  ACarlaWheeledVehicle *GetPossessedVehicle()
  {
    return Vehicle;
  }

  const ACarlaWheeledVehicle *GetPossessedVehicle() const
  {
    return Vehicle;
  }

  /// @}
  // ===========================================================================
  /// @name Control options
  // ===========================================================================
  /// @{

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetStickyControl(bool bEnabled)
  {
    bControlIsSticky = bEnabled;
  }

  /// @}
  // ===========================================================================
  /// @name Road map
  // ===========================================================================
  /// @{

public:

  void SetRoadMap(URoadMap *InRoadMap)
  {
    RoadMap = InRoadMap;
  }

  UFUNCTION(Category = "Road Map", BlueprintCallable)
  URoadMap *GetRoadMap()
  {
    return RoadMap;
  }

  /// @}
  // ===========================================================================
  /// @name Random engine
  // ===========================================================================
  /// @{

public:

  UFUNCTION(Category = "Random Engine", BlueprintCallable)
  URandomEngine *GetRandomEngine()
  {
    check(RandomEngine != nullptr);
    return RandomEngine;
  }

  /// @}
  // ===========================================================================
  /// @name Autopilot
  // ===========================================================================
  /// @{

public:

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  bool IsAutopilotEnabled() const
  {
    return bAutopilotEnabled;
  }

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetAutopilot(bool Enable, bool KeepState = false)
  {
    if (IsAutopilotEnabled() != Enable)
    {
      ConfigureAutopilot(Enable, KeepState);
    }
  }

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void ToggleAutopilot()
  {
    ConfigureAutopilot(!bAutopilotEnabled);
  }

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  bool IsACCEnabled() const
  {
    return bACCEnabled;
  }

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetACC(bool Enable)
  {
    if (IsACCEnabled() != Enable)
    {
      ConfigureACC(Enable);
    }
  }

  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void ToggleACC()
  {
    ConfigureACC(!bACCEnabled);
  }

private:

  void ConfigureAutopilot(const bool Enable, const bool KeepState = false);

  void ConfigureACC(const bool Enable);

  /// @}
  // ===========================================================================
  /// @name Traffic
  // ===========================================================================
  /// @{

public:

  /// Get current speed limit in km/h.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  float GetSpeedLimit() const
  {
    return SpeedLimit;
  }

  /// Set vehicle's speed limit in km/h.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetSpeedLimit(float InSpeedLimit)
  {
    SpeedLimit = InSpeedLimit;
  }

  /// Get traffic light state currently affecting this vehicle.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  ETrafficLightState GetTrafficLightState() const
  {
    return TrafficLightState;
  }

  /// Set traffic light state currently affecting this vehicle.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetTrafficLightState(ETrafficLightState InTrafficLightState)
  {
    TrafficLightState = InTrafficLightState;
  }

  /// Get traffic light currently affecting this vehicle.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  ATrafficLightBase *GetTrafficLight() const
  {
    return TrafficLight;
  }

  /// Set traffic light currently affecting this vehicle.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetTrafficLight(ATrafficLightBase *InTrafficLight)
  {
    TrafficLight = InTrafficLight;
  }

  /// Set a fixed route to follow if autopilot is enabled.
  UFUNCTION(Category = "Wheeled Vehicle Controller", BlueprintCallable)
  void SetFixedRoute(const TArray<FVector> &Locations, bool bOverwriteCurrent = true);

  /// @}

private:

  FVehicleControl TickAutopilotController(const float DeltaTime);

  /// Returns steering value.
  float GoToNextTargetLocation(FVector &Direction);

  /// Returns steering value.
  float CalcStreeringValue(FVector &Direction);

  /// Returns throttle value.
  float Stop(float Speed);

  /// Returns throttle value.
  float Move(float Speed);

  /// PID controller
  float run_step (float current_speed, float target_speed, const float DeltaTime);

  /// ACC system
  float CalculateTargetSpeed(const float Speed, FHitResult &ClosestVehicle, const float DeltaTime, bool &accControl);

  /// @}
  // ===========================================================================
  // -- Member variables -------------------------------------------------------
  // ===========================================================================
  /// @{

private:

  UPROPERTY()
  ACarlaWheeledVehicle *Vehicle = nullptr;

  UPROPERTY()
  URoadMap *RoadMap = nullptr;

  UPROPERTY()
  URandomEngine *RandomEngine = nullptr;

  UPROPERTY(VisibleAnywhere)
  bool bAutopilotEnabled = false;

  UPROPERTY(VisibleAnywhere)
  bool bControlIsSticky = true;

  UPROPERTY(VisibleAnywhere)
  float SpeedLimit = 30.0f;

  UPROPERTY(VisibleAnywhere)
  ETrafficLightState TrafficLightState = ETrafficLightState::Green;

  UPROPERTY(VisibleAnywhere)
  float MaximumSteerAngle = -1.0f;

  UPROPERTY()
  ATrafficLightBase *TrafficLight;

  std::queue<FVector> TargetLocations;

  //UPROPERTY()
  //AVehiclePIDController PIDController = nullptr;
  //std::deque<float> buffer; //buffer for the PID controller
  float velocity_intergal = 0.0f;
  float prev_velocity = 0.0f;
  args_long args_longitudinal; // longitudinal arguments for the PID controller
  UPROPERTY(VisibleAnywhere)
  bool bACCEnabled = false;
};
