package org.firstinspires.ftc.teamcode.RobotFunction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ConveyorBelt {
    public DcMotor conveyor;

    public void init(HardwareMap hwMap) {
        conveyor = hwMap.get(DcMotor.class, "ConveyorBelt");
        conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void run(double power) {
        conveyor.setPower(power);
    }

    public double getPower() {
        return conveyor.getPower();
    }
}