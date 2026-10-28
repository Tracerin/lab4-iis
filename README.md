# Lab 4 
link: https://youtube.com/shorts/x1GWsBcruCI?si=NtY0X29H3QBXrUzt 
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

## To Run
- Once sketch is uploaded, follow calibration sequence instructions on screen
- Once touch and gyro calibrations have finished, you may rotate the physical cube in your hands to rotate the digital one accordingly
- Touch the touch cap wire to increase the speed
- Touch the bottom left cap wire to increase the fisheye effect (moves camera closer to cube)
- Touching the bottom right cap wire at the same time as either of the two previous actions will cause them to be inverted\

AI generated code was used for this sketch