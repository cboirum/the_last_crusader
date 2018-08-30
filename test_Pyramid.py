#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version by Ken Lauer / sirkne at gmail dot com
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

from framework import *
import spriteUtils
import Assets
PPM = 20
class Pyramid (Framework):
    name="Pyramid"
    def __init__(self):
        super(Pyramid, self).__init__()
        # The ground
        ground = self.world.CreateStaticBody(
                    shapes=b2EdgeShape(vertices=[(-800,0),(800, 0)])
                )

        box_half_size = (0.5, 0.5)
        box_density = 5.0
        box_rows = 20

        x=b2Vec2(-7, 0.75)
        deltaX=(0.5625, 1.25)
        deltaY=(1.125, 0)
        crusaderBase = Assets.Crusader(PPM)
        crusader1 = crusaderBase.add2World(self.world,(80,15))
        #crusader2 = crusaderBase.add2World(self.world,(5,40))
        crusader1.angle=3.1415
        crusader1.ApplyForce((-2000000,1),(-1000,0))
        for i in range(box_rows):
            y = x.copy()

            for j in range(i, box_rows):
                self.world.CreateDynamicBody(
                    position=y,
                    fixtures=b2FixtureDef(
                            shape=b2PolygonShape(box=box_half_size),
                            density=box_density)
                    )

                y += deltaY

            x += deltaX
        pyramids = 3
        for i in range(pyramids):
            x=b2Vec2(-7-20*i, 0.75)
            for i in range(box_rows):
                y = x.copy()
    
                for j in range(i, box_rows):
                    self.world.CreateDynamicBody(
                        position=y,
                        fixtures=b2FixtureDef(
                                shape=b2PolygonShape(box=box_half_size),
                                density=box_density)
                        )
    
                    y += deltaY
    
                x += deltaX

if __name__=="__main__":
     main(Pyramid)
