// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorcontrollerPort=1;
    public static final int ktankdriveMotor1rId= 11;
    public static final int ktankdriveMotor2rId= 12;
    public static final int ktankdriveMotor1lId= 13;
    public static final int ktankdriveMotor2lId= 14;
    public static final int GyroID=21;
    public static final int kintakeRollerId = 31;
    public static final int kintakePivotid = 32;
    public static final boolean LEFT_DriveMOTOR_INVERTED=false;
    public static final boolean RIGHT_DriveMOTOR_INVERTED=true;
    public static final Double DEADBAND= 0.02;
    public static final boolean intakerollerinverted = false;
    public static final boolean intakepivotinverted = false;
    public static final int IntakeBoreID = 0;
    public static final double positionTolerance = 3;
    public static final double pivotkP= 0.2;
    public static final double pivotkI = 0.0;
    public static final double pivotkD = 0.0;
    public static final double INTAKE_OPEN_POSITION=120;
    public static final double INTAKE_CLOSED_POSITION=5;
    public static final double intakerollerspeed= 0.2;
    public static final int kShooterRId = 41;
    public static final int kShooterLId = 42;
    public static final boolean kShooterLInverted= false;
    public static final boolean kShooterRInverted= false;
    public static final double kshooterP= 0.2;
    public static final double kshooterI= 0.0;
    public static final double kshooterD= 0.0;
    public static final double kshooterpositionTolerance = 3;
    public static final double kshootervelocityTolerance = 1;
    public static final double shooterTargetRPM = 300;
    public static final double intakerollerrealesespeed = 0.1;
    public static final double gearRatio = 1/60; //gear ratio of the drive //TO DO
    public static final double raduisOfWheelInMeters = 4; //TO DO 
    public static final double circumferenceOfWheelInMeters = Math.PI*2*raduisOfWheelInMeters;
    public static final double conversionFactor = circumferenceOfWheelInMeters*gearRatio;
    public static final double trackwWidth= 23; //in meters //TO DO
    public static final double kmaxspeedmps = 3;//TO DO
    public static final double kmaxaccmpssqr = 3;//TO DO

    }

