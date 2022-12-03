package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.SignalSleeveDetector;
import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.DelayAction;
import org.firstinspires.ftc.teamcode.auto.actions.FullStopAction;
import org.firstinspires.ftc.teamcode.auto.actions.SetArmAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;
import org.firstinspires.ftc.teamcode.auto.actions.WaitAction;
import org.firstinspires.ftc.teamcode.auto.util.WebcamPipeline;

@Autonomous(name = "South Red Auto")
public class SouthRedAuto extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeHolder.opMode = this;
        WebcamPipeline.clearLastMat();
        Config config = new Config.Builder()
                .setDebugMode(true)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(3.3, 0.001, 0), new PIDCoefficients(400, 0.03, 0))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        ArmPositionAction armPositionAction = new ArmPositionAction(manager);
        ToggleClawAction toggleClawAction = new ToggleClawAction(manager);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(armPositionAction)
                .addLinearPath(new Position(0, 600,  0))
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 1500))
                .addAction(new SetArmAction(manager, 11000))
                .addLinearPath(
                        new Position(0, 1350,  0),
                        new Position(0, 1350,  7 * Math.PI / 4),
                        new Position(145, 1500,  7 * Math.PI / 4)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .addAction(new SetArmAction(manager, 0))
                .addLinearPath(
                        new Position(0, 1350, 7 * Math.PI / 4),
                        new Position(0, 1350, Math.PI / 2)
                )
                .addLinearPath(new Position(-680, 1350, Math.PI / 2))
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .addAction(new SetArmAction(manager, 11000))
                .addLinearPath(
                        new Position(0, 1350, Math.PI / 2),
                        new Position(0, 1350, 7 * Math.PI / 4),
                        new Position(145, 1500, 7 * Math.PI / 4)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .build();

        SignalSleeveDetector detector = new SignalSleeveDetector(manager);
        telemetry.addData("Dots: ", detector.detectOrientation());
        telemetry.update();
        waitForStart();

        pipeline.execute();

    }
}
