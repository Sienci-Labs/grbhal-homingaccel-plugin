# Sienci Homing Acceleration Plugin

grblHAL plugin that applies a per-axis acceleration override while an axis is
homing. It is intended for closed-loop sensorless homing, which needs lower
acceleration.

The normal acceleration is restored when the homing cycle completes.

## Settings

The plugin adds one setting per enabled axis:

| Axis | Setting |
| --- | --- |
| X | `$280` |
| Y | `$281` |
| Z | `$282` |
| A | `$283` |

Values are in `mm/sec^2`. Set a value to `0` to retain that axis's normal
acceleration during homing.

## Integration

Include the plugin's `CMakeLists.txt` in the driver's build configuration and
define `SIENCI_HOMING_ACCEL_ENABLE=1` for the firmware target.

Add this guarded entry to the driver's `plugins_init.h`:

```c
#if SIENCI_HOMING_ACCEL_ENABLE
    extern void sienci_homing_acceleration_plugin_init (void);
    sienci_homing_acceleration_plugin_init();
#endif
```
