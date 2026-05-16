# 2026 Robot Code

This repository is for the main robot code for 2026 which is a swerve drive robot

## Structure

* `src/main/java/frc/robot` is the java code folder
  *  `commands` is where all of the commands are but we don't use it
      *  `Autos.java` shows how to create a command with a static method
      *  `ExampleCommand.java` is an example of how to define a command class
  *  `subsystems` is where all of the subsystem classes are
      *  `ClimberSubsystem.java` controls the climber motors and provides climber commands
      *  `ExampleSubsystem.java` is an example of how to create a subsystem including Command factory methods
      *  `ShooterSubsystem.java` controls the shooter, indexer, and floor motors and provides commands for them
      *  `SwerveSubsystem.java` sets up YAGSL for the swerve drive and provides commands to drive and align the robot
      *  `VisionSubsystem.java` sets up PhotonVision
  *  `Constants.java` is for constant values, we don't use this much
  *  `Main.java` is a special file where the Java main method gets set up, we only use it to make our Robot class run
  *  `Robot.java` is the main file that controls the robot. It has the init and periodic methods that are used to run our code during different parts of the game like Teleop and Auto, as well as simulation and test modes
  *  `RobotContainer.java` sets up all of the subsystems and controls
  *  `XBoxConatiner.java` gives action names to different controller inputs so they are easier to work with like driveX
* `src/main/deploy` is the folder for non-java files that need to be copied to the robot
  *  `pathplanner` is for path planner
      *  `autos` contains the autos
      *  `paths` contains the paths used to help build the autos
      *  `navgrid.json` configures where the robot is allowed to path
      *  `settings.json` contains general settings and info about the robot
  *  `swerve` is where the YAGSL swerve configuration goes
      *  `modules` are the individual swerve drive modules
      *  `controllerproperties.json`
      *  `swervedrive.json` is for the general swerve setup including the module names and gyro
  *  `example.txt` is just an example with info about how the deploy folder works
  *  `outputFun.chrp` allows playing specific tones from the motors
*  `vendordeps` contains all of the vendor dependency version JSON files


## Dependencies

* WPILib
* [YAGSL](https://yet-another-software-suite.github.io/YAGSL)
* [YAMS](https://github.com/Yet-Another-Software-Suite/YAMS)

## Building on School Computers

You need to create a file in this folder called `gradle.properties` and add

```
systemProp.javax.net.ssl.trustStoreType=Windows-ROOT
```

so that Gradle will use the school certificates and download vendor libraries correctly

## CAN ID Set up

|Position|Drive CAN ID|Rotation CAN ID|Encoder CAN ID|
|-|-|-|-|
|Front Left|1|2|3|
|Front Right|4|5|6|
|Back Right|7|8|9|
|Back Left|10|11|12|

Gyro: 13


## Robot orientation

```
       FRONT +X
L  ┌---┐     ┌---┐
E  |   |     |   |
F  |   └-----┘   |
T  |             |
+  |             |
Y  └-------------┘
```