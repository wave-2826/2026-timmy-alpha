package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimDeviceLogger {
    static List<Integer> deviceHandles = new ArrayList<>();

    public static void init() {
        SimDeviceSim.registerDeviceCreatedCallback("", (name, handle) -> {
            deviceHandles.add(handle);
        }, true);
        SimDeviceSim.registerDeviceFreedCallback("", (name, handle) -> {
            deviceHandles.remove((Integer) handle);
        }, true);
    }

    public static void update() {
        for(var handle : deviceHandles) {
            var simDevice = new SimDeviceSim(handle);
            var name = simDevice.getName();
            var valueInfos = simDevice.enumerateValues();
            for(var valueInfo : valueInfos) {
                var path = "SimDevices/" + name + "/" + valueInfo.name;
                if(valueInfo.value.getType() == HALValue.kBoolean) {
                    Logger.recordOutput(path, valueInfo.value.getBoolean());
                } else if(valueInfo.value.getType() == HALValue.kDouble) {
                    Logger.recordOutput(path, valueInfo.value.getDouble());
                } else if(valueInfo.value.getType() == HALValue.kInt) {
                    Logger.recordOutput(path, valueInfo.value.getLong());
                } else if(valueInfo.value.getType() == HALValue.kLong) {
                    Logger.recordOutput(path, valueInfo.value.getLong());
                } else if(valueInfo.value.getType() == HALValue.kEnum) {
                    var options = SimDeviceDataJNI.getSimValueEnumOptions(valueInfo.handle);
                    Logger.recordOutput(path, options[(int) valueInfo.value.getLong()]);
                } else {
                    Logger.recordOutput(path, "Unknown value type");
                }
            }
        }
    }
}
