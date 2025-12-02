package org.firstinspires.ftc.teamcode.DriverCode.FieldCentric;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunction.ConveyorBelt;
import org.firstinspires.ftc.teamcode.RobotFunction.Intake;
import org.firstinspires.ftc.teamcode.RobotFunction.OuttakeLauncher;

@TeleOp(name = "FieldCentricRun", group = "Op")
public class FieldCentricRun extends OpMode {
    FieldCentricInitialization drive = new FieldCentricInitialization();
    Intake intake = new Intake();

    OuttakeLauncher outtakeLauncher = new OuttakeLauncher();
    ConveyorBelt belt = new ConveyorBelt();

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        outtakeLauncher.init(hardwareMap);
        belt.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Field-centric drive control
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);

        // Run subsystems automatically
        intake.run(1.0);       // Intake always on
        outtakeLauncher.run(-1.0);
        outtakeLauncher.setServoPos(1);
        belt.run(-1.0);         // Conveyor always on

        // Telemetry feedback
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Outtake Power", outtakeLauncher.getMotorPower());
        telemetry.addData("Outtake Servo", outtakeLauncher.getServoPos());
        telemetry.addData("Conveyor Power", belt.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.run(0);
        outtakeLauncher.run(0);
        belt.run(0);
    }
}