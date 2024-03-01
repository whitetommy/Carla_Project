import carla

def simulate_sudden_unintended_acceleration(vehicle, brake_status):
    """
    Simulates a sudden unintended acceleration scenario where the vehicle accelerates
    while the brake is supposedly engaged.
    """
    global is_sudden_acceleration
    # 급발진 상황을 활성화
    is_sudden_acceleration = True
    
    # Forcefully increase throttle and RPM while keeping the brake engaged
    throttle = 1.0  # Max throttle
    rpm = 5000  # High RPM to simulate acceleration
    brake = brake_status  # Keep the brake status as received

    control = carla.VehicleControl(throttle=throttle, steer=0.0, brake=brake)
    vehicle.apply_control(control)

    # set red light of brake in the SUA situaiton.
    vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Brake))

    # Update visual feedback or print statement to indicate SUA is happening
    global str  # Use the global variable to update brake status
    str = "SUA Active!"  # Update the text to indicate SUA
    print("Sudden Unintended Acceleration is simulated!")