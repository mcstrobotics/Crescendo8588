# stuff we need to do for indexing

set up boilerplate (look at example subsystem, intake subsystem, and the old code for references, as well as their website if needed)

take in the motors

#### make some basic functions
- indexing in
- indexing out
- indexing stop

^ any constants should be defined in Constants.java

#### testing
- for each function make it print something to identify when its called for testing purposes
- bind them to buttons in RobotContainer.java > configureBindings()
- test it (talk to me for help)

#### add telemetry with SmartDashboard
- what state it is in
- for each motor:
    - temprature
    - current draw
    - power draw
---
### after we are done with all of this, we will show it to Val and take his feedback
this approach just uses 2 inputs and based on that makes inte indexing take it in and take it out

### what we will do next
- PIDS + dashboard
- figuring out a system to make sure the note is in the right place