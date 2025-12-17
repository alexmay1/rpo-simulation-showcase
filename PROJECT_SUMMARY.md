# RPO Simulation Project Summary

## What This Project Does

This is a **Rendezvous and Proximity Operations (RPO) simulation package** that simulates rendezvous and proximity operations between a chaser and target satellite in orbit. It provides a complete simulation framework with:

- **Orbital Dynamics**: Realistic orbital mechanics with multiple gravity models
- **Navigation Filters**: Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), and Particle Filter (PF) for absolute state estimation; Pose filter (MUKF) for relative pose estimation
- **Guidance Algorithms**: Multiple guidance modes for rendezvous and approach
- **Sensor Models**: Various sensor types for measurements
- **Actuator Models**: Impulsive and finite burn models
- **Visualization**: Comprehensive plotting and analysis tools

## Project Structure

```
rpo/
├── config/              # Configuration management
├── dynamics/            # Orbital and attitude dynamics
├── guidance/            # Guidance algorithms
├── navigation/          # Navigation filters
├── sensors/             # Sensor models
├── actuators/           # Actuator models
├── visualization/       # Plotting and visualization
├── utils/               # Utility functions
└── sim/                 # Simulation loop
```

## Key Capabilities

### Current Features ✅

1. **Orbital Dynamics**
   - 2-body, J2, and 4x4 spherical harmonic gravity models
   - Solar radiation pressure
   - Atmospheric drag
   - Third-body gravity (Sun and Moon)

2. **Navigation**
   - Extended Kalman Filter (EKF) for absolute state estimation (12-DOF)
   - Unscented Kalman Filter (UKF) for absolute state estimation (12-DOF)
   - Particle Filter (PF) for absolute state estimation (12-DOF)
   - Pose filter (MUKF) for relative pose estimation (6-DOF)
   - Process noise modeling
   - Measurement processing with outlier detection
   - Camera bias estimation with Vasicek process (for EKF, UKF, PF)
   - Range-dependent bias uncertainty scaling

3. **Guidance**
   - Waypoint guidance using Clohessy-Wiltshire equations
   - GEO rendezvous guidance (fast, fuel-optimal, balanced modes)
   - Approach guidance for servicing frame

4. **Sensors**
   - Camera1 (chaser-to-target az/el with bias estimation)
   - Ground station sensor (range/range-rate/az/el with visibility checking)
   - Pose sensor (feature-based simulator) providing relative position and relative attitude measurements in the pose camera frame

5. **Actuators**
   - Impulsive burns (instantaneous velocity changes)
   - Finite burns (constant acceleration over duration)

6. **Attitude Dynamics**
   - Quaternion-based attitude representation
   - Attitude propagation with RK4
   - Gravity gradient torques
   - Support for control torques

7. **Visualization**
   - Relative position/velocity error plots
   - Measurement residual plots
   - Relative orbit plots
   - Time history plots

### Pending Features ⏳

1. **Image Processing**
   - Image generation capability
   - Long-range image processing (target and stars)
   - Near-field image processing
   - AI-based image processing

2. **Advanced Pose Estimation**
   - Full PnP (Perspective-n-Point) solver implementation
   - Feature tracking over time
   - Multi-feature pose estimation

3. **Advanced Guidance**
   - Optimal control guidance
   - Trajectory optimization
   - Collision avoidance

4. **Advanced Sensors**
   - LIDAR sensor
   - Radar sensor
   - Optical flow sensor

5. **Advanced Filters**
   - [x] Unscented Kalman Filter (UKF) ✅
   - [x] Particle Filter (PF) ✅
   - [x] Pose Filter (MUKF) ✅
   - [ ] Multiple model filters

## How to Use

### Run Simulation
```bash
python run_simulation.py
```

### Test Features
```bash
python tests/test_simulation.py
python tests/test_new_features.py
```

### Use Modules
```python
from rpo.config import simulation_config
from rpo.sim import simulation
from rpo.visualization import plotting

# Create configuration
config = simulation_config.SimulationConfig()

# Run simulation
results = simulation.run_simulation(config)

# Generate plots
data = plotting.compute_relative_state_data(...)
plotting.plot_all(data, ...)
```

## State Vectors

### Main EKF State (12-DOF)
- `X[0:3]`: Target position in ECI frame (m)
- `X[3:6]`: Target velocity in ECI frame (m/s)
- `X[6:9]`: Chaser position in ECI frame (m)
- `X[9:12]`: Chaser velocity in ECI frame (m/s)

### Pose Filter State (7-DOF)
- `X[0:3]`: Relative position in target body frame (m)
- `X[3:7]`: Relative quaternion [q0, q1, q2, q3] (scalar-first)

## Coordinate Frames

- **ECI**: Earth-Centered Inertial frame (primary frame)
- **ECEF**: Earth-Centered Earth-Fixed frame
- **RIC**: Radial-In-track-Cross-track frame (local orbital frame)
- **Body Frame**: Vehicle body frame

## Key Design Principles

1. **Modularity**: Each module is independent and testable
2. **Extensibility**: Base classes enable easy extension
3. **Configuration**: Centralized configuration management
4. **Backward Compatibility**: Original functionality preserved
5. **Documentation**: Comprehensive docstrings and documentation

## Extension Points

1. **New Guidance Algorithms**: Inherit from `BaseGuidance`
2. **New Sensors**: Inherit from `BaseSensor`
3. **New Dynamics Models**: Add to `orbital_dynamics.py`
4. **New Filters**: Add to `navigation/` module
5. **New Visualization**: Add to `visualization/plotting.py`

## Documentation

- **README.md**: Overview and quick start
- **ARCHITECTURE.md**: Detailed architecture
- **QUICK_REFERENCE.md**: Quick reference guide
- **PROJECT_STATUS.md**: Current status and pending features
- **DEVELOPMENT_GUIDE.md**: Development guide
- **.cursorrules**: Cursor rules for AI assistants
- **PROJECT_SUMMARY.md**: This file

## Testing

- Unit tests: Test each module independently
- Integration tests: Test module interactions
- Regression tests: Ensure backward compatibility
- Feature tests: Test new features

## Dependencies

- numpy
- matplotlib
- scipy (optional, for advanced features)

## Future Enhancements

1. **Image Processing**: Long-range and near-field image processing
2. **AI Integration**: AI-based image processing and pose estimation
3. **Advanced Filters**: Unscented Kalman Filter, Particle Filter
4. **Advanced Guidance**: Optimal control, trajectory optimization
5. **Advanced Sensors**: LIDAR, radar, optical flow
6. **Advanced Dynamics**: Flexible body dynamics, contact dynamics

## Contact

[Add contact information here]

## License

[Add license information here]

