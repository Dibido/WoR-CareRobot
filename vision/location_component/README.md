# location_component

Scans a video stream for an AGV with cups. Sends the location and expected
arrival time of a found cup to the `controller` node.

## Cup detection

This component scans for cups when the AGV has gone from one side of the screen
to the other. It calculates the location of all cups and whether the cup is
filled or not. Because only one cup can be picked up at a time, it uses the
first detected filled cup. If there is no filled cup, it sends the first
detected cup. A cup is filled when the midpoint has a bright colour (high
saturation).
