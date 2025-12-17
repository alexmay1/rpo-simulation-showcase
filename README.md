# RPO Simulation Package

Rendezvous and Proximity Operations (RPO) simulation package with modular architecture for guidance, navigation, dynamics, sensors, and visualization.

## Overview

This package simulates rendezvous and proximity operations between a chaser and target satellite. It includes:

- **Orbital Dynamics**: 2-body, J2, and 4x4 spherical harmonic gravity models
- **Navigation Filters**: Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), and Particle Filter (PF) for absolute state estimation; Pose filter (MUKF) for relative pose estimation
- **Guidance Algorithms**: Waypoint guidance, MPC guidance, GEO rendezvous guidance, Approach guidance
- **Sensors**: Camera1 (az/el), Ground station sensor (range/range-rate/az/el), Pose sensor (feature-based)
- **Actuators**: Impulsive and finite burn models
- **Attitude Dynamics**: Quaternion-based attitude propagation with gravity gradient torques
- **Visualization**: Comprehensive plotting and analysis tools

## Project Structure

```
rpo/
├── config/              # Configuration management
│   └── simulation_config.py
├── dynamics/            # Orbital and attitude dynamics
│   ├── orbital_dynamics.py
│   ├── attitude_dynamics.py
│   ├── propagator.py
│   └── coordinate_frames.py
├── guidance/            # Guidance algorithms
│   ├── base_guidance.py
│   ├── waypoint_guidance.py
│   ├── mpc_guidance.py
│   ├── geo_rendezvous_guidance.py
│   └── approach_guidance.py
├── navigation/          # Navigation filters
│   ├── ekf.py
│   ├── ukf.py
│   ├── pf.py
│   ├── pose_filter.py
│   └── filter_utils.py
├── sensors/             # Sensor models
│   ├── base_sensor.py
│   ├── camera1.py
│   ├── ground_station_sensor.py
│   ├── pose_sensor.py
│   └── pose_processor.py
├── actuators/           # Actuator models
│   └── thruster.py
├── visualization/       # Plotting and visualization
│   └── plotting.py
├── utils/               # Utility functions
│   ├── constants.py
│   ├── orbital_utils.py
│   ├── time_utils.py
│   └── math_utils.py
└── sim/                 # Simulation loop
    └── simulation.py
```

## Quick Start

### Basic Usage

Run the simulation:
```python
python run_simulation.py
```

### Using the Modules

```python
from rpo.config import simulation_config
from rpo.sim import simulation
from rpo.visualization import plotting

# Create configuration
config = simulation_config.SimulationConfig()

# Run simulation
results = simulation.run_simulation(config)

# Generate plots
data = plotting.compute_relative_state_data(
    results['X_true_log'],
    results['X_est_log'],
    results['P_log'],
    results['log_times'],
    results['n']
)
plotting.plot_all(data, results['meas_log_times'], 
                  results['innovations'], results['S_diagonal'])
```

## Key Features

### 1. Guidance Algorithms

- **Waypoint Guidance**: Clohessy-Wiltshire based waypoint tracking
- **MPC Guidance**: Model Predictive Control for docking
- **GEO Rendezvous**: Multiple modes (fast, fuel-optimal, balanced)
- **Approach Guidance**: Servicing frame approach

### 2. Navigation Filters

- **EKF**: Extended Kalman Filter for absolute state estimation (12-DOF: target + chaser position/velocity)
- **UKF**: Unscented Kalman Filter for absolute state estimation (12-DOF) with nonlinear measurement updates
- **PF**: Particle Filter for absolute state estimation (12-DOF) with configurable number of particles
- **Pose Filter**: Multiplicative Unscented Kalman Filter (MUKF) for 6-DOF relative pose estimation (position + attitude)
- **Bias Estimation**: Camera bias estimation using Vasicek process (available in EKF, UKF, PF)

### 3. Sensors

- **Camera1**: Chaser-to-target azimuth/elevation measurements with bias estimation
- **Ground Station Sensor**: Range, range-rate, azimuth, elevation from ground station with visibility checking
- **Pose Sensor**: Feature-based pose estimation (simulator) for relative pose measurements

### 4. Dynamics

- **Orbital Dynamics**: 2-body, J2, 4x4 spherical harmonic gravity
- **Attitude Dynamics**: Quaternion-based attitude propagation
- **Disturbances**: Solar radiation pressure, atmospheric drag, third-body gravity

### 5. Actuators

- **Impulsive Burns**: Instantaneous velocity changes
- **Finite Burns**: Constant acceleration over duration

## Configuration

Configuration is managed through `SimulationConfig` class:

```python
from rpo.config import simulation_config

config = simulation_config.SimulationConfig()

# Orbital parameters
config.a_target = 42164e3  # GEO altitude
config.e_target = 0.0
config.i_target = np.radians(0.01)

# Simulation parameters
config.T = 20 * 3600  # 20 hours
config.dt = 60  # 60 second logging

# Dynamics settings
config.gravity_model = 'J2'  # '2-body', 'J2', or '4x4'
config.enable_srp = False
config.enable_third_body = False
config.enable_drag = False

# Guidance settings
config.waypoint_guidance = [...]
config.mpc_guidance = {...}
config.guidance_schedule = [...]
```

## Extending the Code

### Adding a New Guidance Algorithm

1. Create a new class inheriting from `BaseGuidance`
2. Implement `compute_delta_v()` and `is_active()` methods
3. Add to `guidance/__init__.py`

### Adding a New Sensor

1. Create a new class inheriting from `BaseSensor`
2. Implement `measure()`, `compute_jacobian()`, and `get_noise_covariance()` methods
3. Add to `sensors/__init__.py`

### Adding a New Dynamics Model

1. Add new acceleration functions to `dynamics/orbital_dynamics.py`
2. Update `compute_acceleration()` function
3. Add configuration options to `SimulationConfig`

## Future Enhancements

Planned features:
- Image generation and processing
- Advanced pose estimation (PnP solver)
- Long-range image processing
- Near-field image processing
- AI-based image processing
- Additional guidance modes
- Extended attitude control

## Dependencies

- numpy
- matplotlib
- cvxpy (for MPC optimization)
- scipy (optional, for advanced features)

## Testing

Run tests:
```bash
python tests/test_simulation.py      # Test simulation
python tests/test_new_features.py    # Test new features
```

Or install the package and run:
```bash
pip install -e .
python tests/test_simulation.py
```

## Documentation

For detailed documentation, see:

- **README.md**: This file - Overview and quick start
- **ARCHITECTURE.md**: Detailed architecture documentation
- **PROJECT_SUMMARY.md**: High-level project summary
- **MAKING_PUBLIC.md**: Instructions for making this showcase folder public on GitHub

Note: This showcase folder contains only documentation and visualization materials. The full source code is in the private repository.

## License

[Add license information here]

## Authors

[Add author information here]

