Instructions for tuning the turret (for now, idk)

## Characterize losses
- Make sure the turret is solidly mounted to a surface which won't vibrate
- Connect to all turret motors via Rev Hardware Client
- Ensure conversion factors on every motor cause encoder velocity to be in rad/s _at the motor_ (primary encoder > velocity conversion factor = `0.1047198`)
    - The code resets this to something else! It's important to change though.
- Set motor PIDs appropriately so you can use them all for velocity control. Something like (0.002, 0, 0, 0.0019) for azimuth and (0.004, 0, 0, 0.0019) for other motors should work fine.
- Set up telemetry appropriately
    - Set the graph to 10 seconds
    - For each of the 4 motors, turn on "Current" and "Primary Encoder Velocity"
    - This is going to sound _really_ stupid, but it's REV's fault, not ours. For each motor, turn on a different warning being graphed so we can uniquely identify them in the exported data.
        - Top flywheel motor: "Brownout Sticky Warning"
        - Bottom flywheel motor: "Brownout Warning"
        - Azimuth motor: "Ext EEPROM Sticky Warning"
        - Hood motor: "Ext EEPROM Warning"
- Tune flywheel motors
    - Hold the other turret motors in place
    - Run both motors at the same percent - the **lowest which causes the flywheel to move**
    - Collect telemetry until there's a full graph of reasonably stable data
    - Stop the graph before stopping the motors so no slowdown is recorded
    - Pause and export as a CSV into the `misc/turretTuning/data` directory with the name `flywheel_slow.csv`
    - With the same testing process an dtelemetry, run the motors at **50% power** and export into `flywheel_fast.csv`
- Tune azimuth
    - Redo flywheel tuning with just the azimuth motor on
    - Save to `azimuth_slow.csv` and `azimuth_fast.csv`
- Tune hood
    - Redo flywheel tuning with just the hood motor on
    - Save to `hood_slow.csv` and `hood_fast.csv`
- Characterize plate frictions
    - Using velocity control in REV Hardware Client, run all 4 motors at the same speed (so the three rings run identically) of **100 rad/s**
    - Collect data and save to `all_rings_fast.csv`
    - Run all 4 motors at **10 rad/s**, collect, and save to `all_rings_slow.csv`
- Characterize bottom pulley friction
    - Run the top two rings at **100 rad/s**, collect, and save to `bottom_static_fast.csv`
    - Run the top two rings at **10 rad/s**, collect, and save to `bottom_static_slow.csv`
- Characterize top pulley friction
    - Run the bottom two rings at **100 rad/s**, collect, and save to `top_static_fast.csv`
    - Run the bottom two rings at **10 rad/s**, collect, and save to `top_static_slow.csv`
- Run `analyze_data.py`: `just tune-turret`

## Zero position
todo