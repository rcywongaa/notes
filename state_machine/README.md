# State Machine
An experimental implementation of a state machine that aims to keep state
transition logic local to the states only.

State transition is accomplished overwriting itself with the state-to-be.

## Ref
- Passing constructors is forbidden: <https://stackoverflow.com/a/954565/3177701>
  - HOWEVER, with lamda, it is possible: <https://stackoverflow.com/a/41083141/3177701>
- Commiting suicide is OK: <https://stackoverflow.com/questions/3150942/is-delete-this-allowed>

## TODO
Avoid commiting suicide, pass constructor via lambda and have controller handle
state construction & destruction
