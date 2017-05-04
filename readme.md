This is a fork of the Gamebuino repo.

So far this fork has the following branches:
NEW_DISPLAYS
-> Adds the ability to switch the display driver
-> Adds support for SSD1331 and SSD1332 OLED screens
--> Since these displays are unlike the 1bit original display, some modes have been added to settings such as
---> COLOR_SCREEN_OVERRIDE -- This lets you specify a color to draw the 1 bit pixels at instead of white, its in 8-bit bgr format
---> COLOR_SCREEN_OVERRIDE_CYCLE -- This is a cycling color effect, kinda trippy, needs improvement
---> DISPLAY_EMULATE_84x48 -- This makes all the old original gamebuino games play nice with the screen size and centers it within the display
====
Some notes:

GRAY is disabled, it's not possible to add GRAY support in the same fashon, GRAY turns on and off the same pixel every other frame, which on the original display looks cool with the right contrast, on the OLED is isn't so good.  So it can be turned on as a white pixel or off, OFF seems to be the right approach for most games.  And since the screen is blit all at the same time, the framebuffer only stores 1 bit info, so its not possible right now to know where to draw GRAY.

These drivers still use the 1bit frame buffer, there isn't enough memory in the 328p to store a whole framebuffer with a larger bit depth (maybe 2 bit?), so I made a driver that doesn't use the frame buffer and draws the screen instantly, this causes a flicker from clearing the screen and blitting the objects again.  The data sheet implys tying the FR pin to sync the output to the refresh of the OLED to prevent this.  I don't have that pin broken out so that code has been archived for now.
====




===================================

Welcome to the Gamebuino official repository !
Please visit [the website](http://gamebuino.com) for more information.
The [Getting Started](http://gamebuino.com/wiki/index.php?title=Getting_started) page explains how to use your Gamebuino.

![](http://gamebuino.com/wp-content/uploads/2013/10/gamebuino.gif)

Gamebuino Library License
-------------------------

(C) Copyright 2014 Aurélien Rodot. All rights reserved.

The Gamebuino Library is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

Included Libraries License
--------------------------

Some libraries other than the Gamebuino Library are included in the sub-folder Libraries for the sake of convenience. You will find the corresponding licence in their respective folders.
