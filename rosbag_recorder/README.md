# Rosbag Recorder
Simple wrapper for rosbag recorder

## Params
- `topics`: List of topics to record
- `max_size_G`: Max size per bag (in Gb)
- `max_splits`: Max number of splits (total max size = `max_splits * max_size_G`)
