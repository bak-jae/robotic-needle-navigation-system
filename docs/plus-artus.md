# Telemed ArtUS and Plus

## Verified acquisition environment

- Ultrasound device: Telemed ArtUS
- Acquisition computer: Windows
- Plus package: `PlusApp-2.8.0.20190617-Telemed-Win32.exe`
- Official release: [Plus Toolkit 2.8.0](https://github.com/PlusToolkit/PlusLib/releases/tag/Plus-2.8.0)
- Previously used asset: [Telemed Win32 installer](https://github.com/PlusToolkit/PlusLib/releases/download/Plus-2.8.0/PlusApp-2.8.0.20190617-Telemed-Win32.exe)

Install Plus and the vendor driver on the Windows acquisition computer. They are not
distributed in this repository. The Plus XML must match the ArtUS device identifier,
video source, and OpenIGTLink port. After validation on the device, a sanitized example
XML may be added under `config/` without credentials, patient data, or machine-specific
absolute paths.

## Data flow

```text
Telemed ArtUS -> PlusServer (Windows) -> OpenIGTLink IMAGE
              -> Slicer OpenIGTLinkIF (Ubuntu) -> scalar volume
              -> NeedleNavigation Max projection -> ROS 2 Image
```

`192.168.0.102` is an example sender address from the verified setup, not a fixed system
requirement. Record the actual Windows sender address and PlusServer port in the ignored
`config/system.local.yaml`, and enter the same values in the Slicer OpenIGTLinkIF
connector.

## Connect Slicer

1. Start PlusServer on the Windows computer using the validated Telemed configuration.
2. Confirm that the Ubuntu and Windows computers can communicate over the network.
3. In Slicer **OpenIGTLinkIF**, create a Client connector.
4. Enter the Windows sender address and PlusServer port, then activate the connector.
5. Confirm that a scalar volume node is updating.
6. Select that node as **Input ultrasound volume** in Needle Navigation.

The incoming volume does not need to be named `Image_Reference`. That name is only the
automatic default; the UI selector binds the selected scalar volume at runtime.

## Projection characteristics

The verified input array shape was `(Z, 512, 512)`, with an observed Z size of 1. The
published image is the 512 x 512 `mono8` result of `max(axis=0)`. If Z contains multiple
slices, each output pixel is the maximum value at that pixel location across all slices,
producing a maximum-intensity projection. This removes depth information, so a future
detection study should separately determine whether it also requires the original 3D
volume.
