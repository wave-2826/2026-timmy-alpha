# Motor Current

## Supply versus stator

One of the biggest points of confusion we've learned a lot about this year is how motor current works. There's this vague concept of "motor current" being split into "stator current" and "supply current" throughout the ecosystem, but many vendors - and WPILib itself - don't make a clear distinction.

Supply current is the electrical current drawn from the power source (PDH) to the motor controller, while stator current is the current flowing through the windings of the motor itself.

Stator current is typically higher than supply current because the motor controller converts voltage and current efficiently, but not perfectly. The supply current includes losses (like heat) and is affected by the controller's efficiency. In brushless motors, the stator current directly determines torque, while supply current reflects overall power consumption.

Therefore, **Stator current is almost always what you want**. Supply current is only really relevant for tracking PDH current limits/battery power consumption.

At stall, the motor duty cycle is typically very low because large amounts of stator current are used at a low duty cycle. As the motor approaches free speed, the duty cycle will increase and difference between these currents will decrease. To calculate supply current, we use the formula:
$$I_{supply} = \frac{V_{applied}}{V_{battery}} \cdot I_{stator}$$

Modulation in both commutation phases and switching frequency makes these current measurements... complicated in brushless motors. The motor controller rapidly switches the output voltage to control the motor, so one would expect the current to spike wildly, but the motor windings [act as large inductors](https://www.chiefdelphi.com/t/rev-robotics-2022-product-releases-updates/401460/28?u=brodye). This inductive property smooths out the current drawn from the supply, making it relatively stable even though the controller is switching quickly. Still, the sum of all three phases' currents is zero, so in a brushless motor, the stator current doesn't reflect any "real" reading. With field-oriented control, it's the q-axis of the current vector that actually determines the torque, so the controller does fancy math to convert the three-phase currents into a single "effective" current that it uses for control. For trapezoidal commutation (like rev sparks use), the current returned is even more decoupled from the actual current in the motor but effectively captures the power being converted into torque.

[This](https://scolton.blogspot.com/2009/11/everything-you-ever-wanted-to-know.html) is a fantasic summary of how brushless motor control works and simplifies to a brushed model.

All vendors handle this differently, unfortunately.
- In WPILib, all currents used in `DCMotor` are stator currents. This makes sense, since the model is a simplification brushed DC motors to an extent.
- For REV, the ambiguously-named `getOutputCurrent()` [returns a stator current](https://www.chiefdelphi.com/t/discrepancy-in-spark-max-pdh-current-reporting/408303/2) and their current control mode takes a stator current.
- For CTRE, things are a bit clearer. `getStatorCurrent()` provides a stator current status signal, while `getSupplyCurrent()` provides a supply current status signal. Torque current FOC control takes a stator current.