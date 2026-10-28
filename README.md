# Lab 4 
link: 
## My interactive system
- Cube cube
- Renders a cube on a display that rotates as you rotate the real cube (pretend it is not a sad rectangular prism)
- 3 touch sensors determine the speed and the curvature effect of the cube render
- Try to line up the speed of rotation with the cube you are physically rotating

## How it works
- Using a gyro module, calibrates the initial gyro position / noise, setting a deadzone
- Gyro measures how fast the board is turning and updates the cube rotation every frame accordingly
- Similar calibration sequence on cap wires as previous labs
- Cube is rendered as 8 points with all 12 edges always visible

AI generated code was used for this sketch