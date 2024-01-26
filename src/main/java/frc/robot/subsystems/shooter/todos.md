# stuff we need to do for shooter

set up boilerplate (look at example subsystem, intake subsystem, and the old code for references, as well as their website if needed)

take in the motors

#### make some basic functions
- turn aim up
- turn aim down
- shoot function (takes in power)
- you guys need to brainstorm how we will calculate where the shot will go
    - make a model that takes a bunch of variables and from that determines where it will land
        - visualise in desmos or sm 

^ any constants should be defined in Constants.java

#### testing
- for each function make it print something to identify when its called for testing purposes
- bind them to buttons in RobotContainer.java > configureBindings()
- test it (talk to me for help)

#### add telemetry with SmartDashboard
- what state shooter is in
- for each motor:
    - temprature
    - current draw
    - power draw
---
### after we are done with all of this, we will show it to Val and take his feedback
we will need to add stuff, this is just a starting off point

### what we will do next
- PIDS + dashboard
- figuring out a system to make sure the note is in the right place