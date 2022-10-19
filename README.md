# ROS-CryoTemplate
ROS code for CryoTempalte


## trajcontrol dependencies



## Communication diagram
```mermaid
sequenceDiagram

    3DSlicer ->> OpenIGTLink: angles
    OpenIGTLink ->> 3DSlicer: Status
    OpenIGTLink ->> 3DSlicer: Motor_Positions
    OpenIGTLink ->> 3DSlicer: Foot_Switch

    OpenIGTLink ->> Main: angles
    Main ->> Communication: Desired_motor_pos
    Main ->> OpenIGTLink: status

    Communication ->> OpenIGTLink: motor_pos
    Communication ->> Main: Foot_switch 
    
    Communication ->> Galil: Desired_motor_pos 
    Galil ->> Communication: read_motor_pos 
    Galil ->> Communication: read_foot_switch 

```

