# Drake

- Models (`.sdf`) that do not have a joint attaching `world` to it will automatically receive a [floating base](https://stackoverflow.com/questions/55711025/how-to-make-sense-of-the-continuous-state-vector/55713199?noredirect=1#comment98127900_55713199)
- Joint pose are specified with respect to `<child>` frame
- Plants should be implemented with proper [Scalar Conversion](https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html)
- Use the following macro to enable proper linking when using templates in `.cc` file
      DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
      class MyClass)
- `<geometry><mesh>` only supports `.obj` objects
