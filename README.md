# 2026 Robot Code

This repository is for the main robot code for 2026 which is a swerve drive robot

## Structure

* `src/main/java/frc/robot` is the java code folder
  *  `subsystems` is where all of the subsystem classes are
  *  `commands` is where all of the commands are

## Dependencies

* WPILib
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