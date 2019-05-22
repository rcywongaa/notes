# Drake

- Models (`.sdf`) that do not have a joint attaching `world` to it will automatically receive a [floating base](https://stackoverflow.com/questions/55711025/how-to-make-sense-of-the-continuous-state-vector/55713199?noredirect=1#comment98127900_55713199)
- Joint pose are specified with respect to `<child>` frame
- Plants should be implemented with proper [Scalar Conversion](https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html)
- Use the following macro to enable proper linking when using templates in `.cc` file
      DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
      class MyClass)
- `<geometry><mesh>` only supports `.obj` objects
- `FixInputPort` requires the context of the corresponding port. The context can be taken with `diagram->GetMutableSubsystemContext(my_subsystem, diagram_context.get())`
  ```
  Context<double>& target_subsystem_context = diagram->GetMutableSubsystemContext(*target_subsystem, diagram_context.get());
  target_subsystem_context.FixInputPort(target_subsystem->get_input_port().get_index(), Vector1d::Zero());
  ```

- Enable multicast without internet connection (drake requires multicast to work)
      sudo ip route add 224.0.0.0/4 dev lo
