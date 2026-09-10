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
    SLICES["Red: original + navigation<br/>Yellow: image + red detected tip"]
    MODEL["ProbeToImage + NeedleMoveEncoder<br/>MRML transforms/model"]
    ROS2EXT["SlicerROS2<br/>C++ extension"]
    OIGTL --> PY
    REG --> MODEL
    PY <--> MODEL
    PY <--> ROS2EXT
    PY --> SLICES
  end

  subgraph ROS["Independent ROS 2 processes"]
    BRIDGE["/epos_motion_bridge_node<br/>C++"]
    DETECT["/needle_tracker_node<br/>SmallUNet Python process"]
  end

  subgraph HW["Motor hardware"]
    EPOS["EPOS4<br/>theta + linear"]
  end

  PLUS -- "OpenIGTLink IMAGE<br/>IP/port configurable" --> OIGTL
  ROS2EXT -- "projection + encoder pixel geometry" --> DETECT
  DETECT -- "numeric result + mono8 preview" --> ROS2EXT
  ROS2EXT -- "/needle/cmd/*" --> BRIDGE
  BRIDGE -- "/needle/state/*" --> ROS2EXT
  BRIDGE -- "EPOS Command Library / CAN" --> EPOS
  EPOS -- "encoder counts" --> BRIDGE
```

The Slicer Python module owns visualization, user interaction, recording, projection, and
mapping degree/mm state into the MRML transform. The bridge is a separate ROS 2 C++ process
and owns EPOS unit conversion and hardware I/O. PlusServer is a separate Windows program.

Detection remains an independent ROS 2 node so CUDA/PyTorch work cannot block the Slicer UI.
The node subscribes to images and encoder-derived pixel geometry, but publishes only detection
results and a preview. There is deliberately no detector-to-bridge motor command path. Any
future closed-loop control must be added as a separate, safety-reviewed layer with explicit
limits and watchdog behavior.

The detector computes in canonical 512×512 coordinates. Its preview is restored to the native
input dimensions, and Slicer copies the source volume's IJK-to-RAS matrix and parent transform.
Red and Yellow then share an identical SliceToRAS matrix, preventing aspect, registration, or
90-degree rotation differences.
