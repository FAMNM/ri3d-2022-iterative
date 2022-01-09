# ri3d-2022-iterative

This is the iterative version of FAMNM's (First Alumni and Mentors Network at Michigan's) Ri3D code for the 2022 FRC season. The command based version of this code (found in the ri3d-2022-command repository) is functionally equivalent to this code. Our goal as a team is to help FRC teams, and this code can be used as reference.

This code contains vision processing capabilities that will ideally track the cargo on the field. By tuning the line that says "Core.inRange(.......", you can select what color to track. The outputs are normalized X-Y coordinates relative to the center of the camera's field of view (i.e., if the target is on the far left and vertically centered, visionXLocation will be -1.0 and visionYLocation will be 0.0). This functionality can be very resource intensive, and may slow down the roboRIO so much that it crashes. To ensure this doesn't happen, we have selected a low camera resolution and minimized the amount of processing we do.
