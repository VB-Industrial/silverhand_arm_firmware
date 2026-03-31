# TODO

## Mismatch Recovery vs Upper Position Loop

Current issue:
- when encoder/TMC mismatch is detected, the firmware may resync the internal angle/offset
- the upper controller can still hold a position target like `position=0, velocity=0`
- after the resync, the upper loop sees a state jump and starts driving the joint back to its target
- this can cause oscillation or shaking of the manipulator

What to revisit later:
- define the expected behavior of mismatch recovery while an external position loop is active
- choose between: hard fault, temporary command freeze, soft state correction, or another recovery strategy
- make sure recovery does not create a state jump that destabilizes the upper controller
