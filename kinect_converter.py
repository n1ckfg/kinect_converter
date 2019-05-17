'''
kinect_converter.py
Copyright (c) 2019 Nick Fox-Gieg Animation
http://fox-gieg.com

The MIT License <http://www.opensource.org/licenses/mit-license.php>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

from math import tan


class KinectConverter(object):

    def __init__(self, model="Kinect"):
        self.horizontalFov = 0
        self.verticalFov = 0
        self.resolutionX = 0
        self.resolutionY = 0
        self.maxDepthVals = 0
          
        self.xzFactor = 0
        self.yzFactor = 0
        self.coeffX = 0
        self.coeffY = 0
        self.halfResX = 0
        self.halfResY = 0

        self.setModel(model)
        self.init()

    def init(self):
        self.xzFactor = tan(self.horizontalFov / 2) * 2
        self.yzFactor = tan(self.verticalFov / 2) * 2
        self.halfResX = self.resolutionX / 2
        self.halfResY = self.resolutionY / 2
        self.coeffX = float(self.resolutionX) / self.xzFactor
        self.coeffY = float(self.resolutionY) / self.yzFactor

    def convertDepthToWorld(self, depthX, depthY, depthZ):
        depthZ -= 255
        normalizedX = depthX / self.resolutionX #depthX / self.resolutionX - 0.5
        normalizedY = depthY / self.resolutionY #0.5 - depthY / self.resolutionY
     
        pWorldX = normalizedX * depthZ * self.yzFactor * -(float(self.resolutionX)/float(self.resolutionY))
        pWorldY = normalizedY * depthZ * self.yzFactor
        pWorldZ = (depthZ / 255) * self.maxDepthVals
        
        return (pWorldX, pWorldY, pWorldZ)
  
    def convertWorldToDepth(self, worldX, worldY, worldZ):
        pDepthX = self.coeffX * worldX / worldZ + self.halfResX
        pDepthY = self.halfResY - self.coeffY * worldY / worldZ
        pDepthZ = worldZ
        
        return (pDepthX, pDepthY, pDepthZ)

    def setModel(self, model="Kinect"):
        if (model == "Kinect4_Narrow_Unbinned"):
            self.resolutionX = 640
            self.resolutionY = 576
            self.horizontalFov = 75.0
            self.verticalFov = 65.0  
            self.maxDepthVals = 2047 # ?
        elif (model == "Kinect4_Narrow_Binned"):
            self.resolutionX = 320
            self.resolutionY = 288
            self.horizontalFov = 75.0
            self.verticalFov = 65.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "Kinect4_Wide_Unbinned"):
            self.resolutionX = 1024
            self.resolutionY = 1024
            self.horizontalFov = 120.0
            self.verticalFov = 120.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "Kinect4_Wide_Binned"):
            self.resolutionX = 512
            self.resolutionY = 512
            self.horizontalFov = 120.0
            self.verticalFov = 120.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "Kinect2"):
            self.resolutionX = 512
            self.resolutionY = 424
            self.horizontalFov = 70.6
            self.verticalFov = 60.0  
            self.maxDepthVals = 8191 # 13-bit
        elif (model == "Xtion"):
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 58.0
            self.verticalFov = 45.0   
            self.maxDepthVals = 2047 # ?      
        elif (model == "Structure"):
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 58.0
            self.verticalFov = 45.0 
            self.maxDepthVals = 2047 # ?       
        elif (model == "StructureCore_4:3"):
            self.resolutionX = 1280
            self.resolutionY = 960
            self.horizontalFov = 59.0
            self.verticalFov = 46.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "StructureCore_16:10"):
            self.resolutionX = 1280
            self.resolutionY = 800
            self.horizontalFov = 59.0
            self.verticalFov = 46.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "Carmine1.09"): # short range
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 57.5
            self.verticalFov = 45.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "Carmine1.08"):
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 57.5
            self.verticalFov = 45.0 
            self.maxDepthVals = 2047 # ?
        elif (model == "RealSense415"):
            self.resolutionX = 1280
            self.resolutionY = 720
            self.horizontalFov = 64.0
            self.verticalFov = 41.0
            self.maxDepthVals = 2047 # ?
        elif (model == "RealSense435"):
            self.resolutionX = 1280
            self.resolutionY = 720
            self.horizontalFov = 86.0
            self.verticalFov = 57.0        
            self.maxDepthVals = 2047 # ?
        else:  # Kinect
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 58.5
            self.verticalFov = 46.6
            self.maxDepthVals = 2047 # 11-bit