package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.DelayAction;
import org.firstinspires.ftc.teamcode.auto.actions.FullStopAction;
import org.firstinspires.ftc.teamcode.auto.actions.SetArmAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;
import org.firstinspires.ftc.teamcode.auto.actions.WaitAction;
import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.WebcamPipeline;

@Autonomous(name="South Blue Auto")
public class SouthBlueAuto extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        OpModeHolder.opMode = this;
        WebcamPipeline.clearLastMat();
        Config config = new Config.Builder()
                .setDebugMode(false)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(1.3, 0.002, 0), new PIDCoefficients(550, 0.7, 0))
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
                        new Position(0, 1350,  Math.PI / 4),
                        new Position(-145, 1500,  Math.PI / 4)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .addAction(new SetArmAction(manager, 0))
                .addLinearPath(
                        new Position(0, 1350, Math.PI / 4),
                        new Position(0, 1350, 3 * Math.PI / 2)
                )
                .addLinearPath(new Position(680, 1350, 3 * Math.PI / 2))
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .addAction(new SetArmAction(manager, 11000))
                .addLinearPath(
                        new Position(0, 1350, 3 * Math.PI / 2),
                        new Position(0, 1350, Math.PI / 4),
                        new Position(-145, 1500, Math.PI / 4)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .build();

        waitForStart();

        pipeline.execute();
    }
}
