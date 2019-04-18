# Drake

- Models (`.sdf`) that do not have a joint attaching `world` to it will automatically receive a [floating base](https://stackoverflow.com/questions/55711025/how-to-make-sense-of-the-continuous-state-vector/55713199?noredirect=1#comment98127900_55713199)
- Joint pose are specified with respect to `<child>` frame
