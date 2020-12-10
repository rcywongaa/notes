# Gazebo TF
Subscribes to Gazebo `/gazebo/model_states` and publishes the static transforms of all objects in the Gazebo world.
Useful for providing simulated ground truth data.

## Params
- `publish_frequency`: Frequency at which to publish updates (`0.0` to use `/gazebo/model_states`'s publish frequency)
