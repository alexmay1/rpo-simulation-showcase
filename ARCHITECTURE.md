# RPO Simulation Architecture

## Architecture Overview

The RPO simulation package uses a modular architecture with clear separation of concerns. Each module is independently testable and extensible.

## Module Descriptions

### Configuration (`config/`)

**Purpose**: Centralized configuration management

**Key Components**:
- `SimulationConfig`: Configuration class with all simulation parameters
- Supports orbital parameters, dynamics settings, guidance parameters, sensor settings

**Usage**:
```python
from rpo.config import simulation_config
config = simulation_config.SimulationConfig()
```

### Dynamics (`dynamics/`)

**Purpose**: Orbital and attitude dynamics models

**Key Components**:
- `orbital_dynamics.py`: Orbital dynamics (gravity, SRP, drag, third-body)
- `attitude_dynamics.py`: Attitude dynamics (quaternion-based)
- `propagator.py`: State propagation (RK4)
- `coordinate_frames.py`: Coordinate frame transformations (RIC basis)

**Key Functions**:
- `compute_acceleration()`: Compute accelerations for target and chaser
- `dynamics()`: Dynamics function for state propagation
- `attitude_dynamics()`: Attitude dynamics function
- `compute_ric_basis()`: Compute RIC (Radial-In-track-Cross-track) basis vectors

### Navigation (`navigation/`)

**Purpose**: Navigation filters for state estimation

**Key Components**:
- `ekf.py`: Extended Kalman Filter for absolute state estimation
- `ukf.py`: Unscented Kalman Filter for absolute state estimation
- `pf.py`: Particle Filter for absolute state estimation
- `pose_filter.py`: Multiplicative Unscented Kalman Filter (MUKF) for relative pose estimation
- `filter_utils.py`: Filter utilities (covariance initialization, process noise)

**Key Classes**:
- `ExtendedKalmanFilter`: Main EKF for 12-DOF state (target + chaser position/velocity)
- `UnscentedKalmanFilter`: UKF for 12-DOF state with nonlinear measurement updates
- `ParticleFilter`: PF for 12-DOF state with configurable number of particles
- `PoseFilter`: MUKF for 6-DOF relative pose (position + attitude) with 18-DOF state (position, velocity, attitude error, angular velocity, gyro bias, CoM offset)

### Guidance (`guidance/`)

**Purpose**: Guidance algorithms for rendezvous and approach

**Key Components**:
- `base_guidance.py`: Base guidance interface
- `waypoint_guidance.py`: Waypoint guidance using Clohessy-Wiltshire equations
- `mpc_guidance.py`: Model Predictive Control guidance
- `geo_rendezvous_guidance.py`: GEO rendezvous with multiple modes
- `approach_guidance.py`: Approach guidance for servicing frame

**Key Classes**:
- `BaseGuidance`: Abstract base class for all guidance algorithms
- `WaypointGuidance`: Waypoint tracking guidance
- `MPCGuidance`: MPC-based guidance
- `GEORendezvousGuidance`: GEO rendezvous with fast/fuel-optimal/balanced modes
- `ApproachGuidance`: Servicing frame approach guidance

### Sensors (`sensors/`)

**Purpose**: Sensor models for measurements

**Key Components**:
- `base_sensor.py`: Base sensor interface
- `camera1.py`: Camera1 sensor for azimuth/elevation (az/el) measurements from chaser to target
- `ground_station_sensor.py`: Ground station sensor (range/range-rate/az/el)
- `pose_sensor.py`: Pose sensor simulator (feature-based)
- `pose_processor.py`: Feature tracking and pose estimation

**Key Classes**:
- `BaseSensor`: Abstract base class for all sensors
- `Camera1`: Camera1 sensor for az/el measurements with bias estimation support
- `GroundStationSensor`: Ground station sensor with visibility checking
- `PoseSensor`: Pose sensor simulator
- `PoseProcessor`: Feature tracking and pose estimation

### Actuators (`actuators/`)

**Purpose**: Actuator models for maneuvers

**Key Components**:
- `thruster.py`: Thruster models (impulsive and finite burns)

**Key Classes**:
- `BurnModel`: Base class for burn models
- `ImpulsiveBurn`: Impulsive burn model
- `FiniteBurn`: Finite burn model with constant acceleration

### Visualization (`visualization/`)

**Purpose**: Plotting and visualization

**Key Components**:
- `plotting.py`: Plotting functions for analysis

**Key Functions**:
- `compute_relative_state_data()`: Compute relative state data for plotting
- `plot_all()`: Generate all plots
- Individual plot functions for different analysis views

### Utilities (`utils/`)

**Purpose**: Utility functions

**Key Components**:
- `constants.py`: Physical constants
- `orbital_utils.py`: Orbital mechanics utilities
- `time_utils.py`: Time and astronomical position utilities
- `math_utils.py`: Mathematical utilities (Legendre polynomials)

### Simulation (`sim/`)

**Purpose**: Main simulation loop

**Key Components**:
- `simulation.py`: Main simulation loop

**Key Functions**:
- `initialize_simulation()`: Initialize simulation state
- `create_dynamics_function()`: Create dynamics function
- `process_waypoint_guidance()`: Process waypoint guidance commands
- `process_maneuvers()`: Process scheduled maneuvers
- `run_simulation()`: Run full simulation

## Data Flow

1. **Initialization**: Configuration loaded, initial states computed
2. **Propagation**: State propagated using dynamics
3. **Measurement**: Sensors generate measurements
4. **Filtering**: Navigation filters process measurements
5. **Guidance**: Guidance algorithms compute delta-V commands
6. **Actuation**: Actuators apply maneuvers
7. **Logging**: States logged for analysis
8. **Visualization**: Plots generated from logged data

## State Vector

### Main EKF State (12-DOF)
- `X[0:3]`: Target position in ECI frame (m)
- `X[3:6]`: Target velocity in ECI frame (m/s)
- `X[6:9]`: Chaser position in ECI frame (m)
- `X[9:12]`: Chaser velocity in ECI frame (m/s)

### Pose Filter State (18-DOF)
- `X[0:3]`: Relative position in RIC frame (m) [x (radial), y (along-track), z (crosstrack)]
- `X[3:6]`: Relative velocity in RIC frame (m/s)
- `X[6:9]`: Relative attitude error (multiplicative small-angle error vector δα, rad)
- `X[9:12]`: Relative angular velocity in chaser body frame (rad/s)
- `X[12:15]`: Chaser gyro bias in chaser body frame (rad/s)
- `X[15:18]`: Target CoM offset in target body frame (m)
- Note: The filter uses multiplicative quaternion updates where the quaternion estimate is stored separately and the covariance tracks the small-angle error vector.

### Extended State (with attitude, 24-DOF)
- `X[0:3]`: Target position in ECI frame (m)
- `X[3:6]`: Target velocity in ECI frame (m/s)
- `X[6:10]`: Target quaternion [q0, q1, q2, q3] (scalar-first, body to inertial)
- `X[10:13]`: Target angular velocity in body frame (rad/s)
- `X[13:16]`: Chaser position in ECI frame (m)
- `X[16:19]`: Chaser velocity in ECI frame (m/s)
- `X[19:23]`: Chaser quaternion [q0, q1, q2, q3] (scalar-first, body to inertial)
- `X[23:26]`: Chaser angular velocity in body frame (rad/s)

## Coordinate Frames

- **ECI**: Earth-Centered Inertial frame
- **ECEF**: Earth-Centered Earth-Fixed frame
- **RIC**: Radial-In-track-Cross-track frame (local orbital frame)
- **Body Frame**: Vehicle body frame
- **Target Body Frame**: Target satellite body frame
- **Chaser Body Frame**: Chaser satellite body frame

## Key Design Decisions

1. **Modular Architecture**: Each module is independent and testable
2. **Base Classes**: Abstract base classes for guidance and sensors enable easy extension
3. **Configuration Management**: Centralized configuration makes it easy to modify parameters
4. **State Management**: Clear state vector definitions for different filter types
5. **Backward Compatibility**: Original functionality preserved while enabling new features

## Extension Points

1. **New Guidance Algorithms**: Inherit from `BaseGuidance`
2. **New Sensors**: Inherit from `BaseSensor`
3. **New Dynamics Models**: Add to `orbital_dynamics.py`
4. **New Filters**: Add to `navigation/` module
5. **New Visualization**: Add to `visualization/plotting.py`

## Testing Strategy

1. **Unit Tests**: Test each module independently
2. **Integration Tests**: Test module interactions
3. **Regression Tests**: Ensure refactored code produces identical results
4. **Feature Tests**: Test new features (finite burns, ground station, etc.)

## Performance Considerations

1. **State Propagation**: Uses RK4 integration for accuracy
2. **Filter Updates**: EKF uses numerical differentiation for state transition matrix
3. **Optimization**: MPC uses CVXPY for optimization
4. **Visualization**: Plotting is separated from simulation loop for performance

## Future Enhancements

1. **Image Processing**: Long-range and near-field image processing
2. **AI Integration**: AI-based image processing and pose estimation
3. **Advanced Filters**: Unscented Kalman Filter, Particle Filter
4. **Advanced Guidance**: Optimal control, trajectory optimization
5. **Advanced Sensors**: LIDAR, radar, optical flow
6. **Advanced Dynamics**: Flexible body dynamics, contact dynamics

