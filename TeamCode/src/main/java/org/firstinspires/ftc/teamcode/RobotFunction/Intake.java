package org.firstinspires.ftc.teamcode.RobotFunction;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends OpMode {
    public DcMotor CarWash;

    public void init(HardwareMap hwMap) {
            CarWash = hwMap.get(DcMotor.class, "CarWash");
            CarWash.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            CarWash.setDirection(DcMotor.Direction.REVERSE);
            CarWash.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void run(double power) {

        CarWash.setPower(power);
    }

    public double getPower() {

        return CarWash.getPower();
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}