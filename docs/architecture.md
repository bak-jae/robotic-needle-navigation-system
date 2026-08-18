# Architecture

```mermaid
flowchart LR
  subgraph WIN["Windows acquisition PC"]
    ARTUS["Telemed ArtUS<br/>ultrasound hardware"]
    PLUS["PlusServer 2.8.0<br/>Telemed Win32"]
    ARTUS --> PLUS
  end

  subgraph SLICER["3D Slicer 5.6.2 process"]
    OIGTL["OpenIGTLinkIF<br/>C++ extension"]
    REG["Fiducial Registration<br/>SlicerIGT"]
    PY["NeedleNavigation<br/>Python module"]
    MODEL["ProbeToImage + NeedleMoveEncoder<br/>MRML transforms/model"]
    ROS2EXT["SlicerROS2<br/>C++ extension"]
    OIGTL --> PY
    REG --> MODEL
    PY <--> MODEL
    PY <--> ROS2EXT
  end

  subgraph ROS["Independent ROS 2 processes"]
    BRIDGE["/epos_motion_bridge_node<br/>C++"]
    DETECT["future needle detector/controller<br/>Python or C++"]
  end

  subgraph HW["Motor hardware"]
    EPOS["EPOS4<br/>theta + linear"]
  end

  PLUS -- "OpenIGTLink IMAGE<br/>IP/port configurable" --> OIGTL
  ROS2EXT -- "/ultrasound/projection/max<br/>sensor_msgs/Image" --> DETECT
  ROS2EXT -- "/needle/cmd/*" --> BRIDGE
  BRIDGE -- "/needle/state/*" --> ROS2EXT
  DETECT -. "future control target" .-> BRIDGE
  BRIDGE -- "EPOS Command Library / CAN" --> EPOS
  EPOS -- "encoder counts" --> BRIDGE
```

The Slicer Python module owns visualization, user interaction, recording, projection, and
mapping degree/mm state into the MRML transform. The bridge is a separate ROS 2 C++ process
and owns EPOS unit conversion and hardware I/O. PlusServer is a separate Windows program.

Future detection should remain an independent ROS 2 node subscribing to the image topic.
Publish detections and proposed targets separately, and place safety/limit validation between
detector output and motor commands. This preserves the current manual behavior while allowing
the detector to be introduced and tested without embedding inference load in Slicer.
