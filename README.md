# USU Mars Rover Team

This is a WIP ROS 2 Jazzy workspace that should enable a user to run the rover currently known as Deimos
If you are interested in contributing, reach out to marsrover@usu.edu

## Execution Instructions

1. If this is your first time running the code, run this first `sudo ./startupScripts/installDependencies.sh`.
2. After you have done that, run `rover.sh` on the rover, and `groundStation.sh` on the groundStation (Update after test runs)

## TODO HERE:

- [ ] Add Execution Instructions
- [ ] Add "Good to Knows"
- [ ] Explain File Structure
- [ ] Add Param Override Instructions
  - [ ] Overriding in controlSketch as well

# Release Notes 0.0.1

- Added Arm Control Script
- Added Drivetrain Control Script
- Updated Arduino code to use Arm Velocity control, and Added a GoHome command
- Added a couple of startupScripts
- Added Yaml for Base Params
