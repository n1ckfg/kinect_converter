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
        # given
        self.horizontalFov = 0
        self.verticalFov = 0
        self.resolutionX = 0
        self.resolutionY = 0
        self.maxBitDepth = 0
        self.minDepth = 0
        self.maxDepth = 0
          
        # calculated
        self.xzFactor = 0
        self.yzFactor = 0

        self.setModel(model)
        self.init()

    def init(self):
        self.xzFactor = tan(self.horizontalFov / 2) * 2
        self.yzFactor = tan(self.verticalFov / 2) * 2

    def remap(self, value, min1, max1, min2, max2):
        range1 = max1 - min1
        range2 = max2 - min2
        valueScaled = float(value - min1) / float(range1)
        return min2 + (valueScaled * range2)

    # per pixel depth in mm
    def convertDepthToWorld(self, x, y, z):
        normX = x / self.resolutionX - 0.5
        normY = 0.5 - y / self.resolutionY
    
        z = abs(255 - z)
        worldZ = self.remap(z, 0, 255, self.minDepth, self.maxDepth)
        worldX = normX * worldZ
        worldY = normY * worldZ

        if (self.resolutionX > self.resolutionY):
            worldX *= (self.resolutionX / self.resolutionY)
        elif (self.resolutionY > self.resolutionX):
            worldY *= (self.resolutionY / self.resolutionX)
        
        return (worldX, worldY, worldZ)
        
    def depthFilter(self, img):
        # TODO filter
        return img

    def setModel(self, model="Kinect"):
        if (model == "Kinect4_Narrow_Unbinned"):
            self.resolutionX = 640
            self.resolutionY = 576
            self.horizontalFov = 75.0
            self.verticalFov = 65.0  
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??
        elif (model == "Kinect4_Narrow_Binned"):
            self.resolutionX = 320
            self.resolutionY = 288
            self.horizontalFov = 75.0
            self.verticalFov = 65.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??
        elif (model == "Kinect4_Wide_Unbinned"):
            self.resolutionX = 1024
            self.resolutionY = 1024
            self.horizontalFov = 120.0
            self.verticalFov = 120.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??
        elif (model == "Kinect4_Wide_Binned"):
            self.resolutionX = 512
            self.resolutionY = 512
            self.horizontalFov = 120.0
            self.verticalFov = 120.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??
        elif (model == "Kinect2"):
            self.resolutionX = 512
            self.resolutionY = 424
            self.horizontalFov = 70.6
            self.verticalFov = 60.0  
            self.maxBitDepth = 4499 # 13-bit
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??
        elif (model == "Xtion"):
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 58.0
            self.verticalFov = 45.0   
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??                  
        elif (model == "Structure"):
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 58.0
            self.verticalFov = 45.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??                   
        elif (model == "StructureCore_4:3"):
            self.resolutionX = 1280
            self.resolutionY = 960
            self.horizontalFov = 59.0
            self.verticalFov = 46.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??            
        elif (model == "StructureCore_16:10"):
            self.resolutionX = 1280
            self.resolutionY = 800
            self.horizontalFov = 59.0
            self.verticalFov = 46.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??            
        elif (model == "Carmine1.09"): # short range
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 57.5
            self.verticalFov = 45.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??            
        elif (model == "Carmine1.08"):
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 57.5
            self.verticalFov = 45.0 
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??            
        elif (model == "RealSense415"):
            self.resolutionX = 1280
            self.resolutionY = 720
            self.horizontalFov = 64.0
            self.verticalFov = 41.0
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??            
        elif (model == "RealSense435"):
            self.resolutionX = 1280
            self.resolutionY = 720
            self.horizontalFov = 86.0
            self.verticalFov = 57.0        
            self.maxBitDepth = 2047 # ?
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??            
        else:  # Kinect
            self.resolutionX = 640
            self.resolutionY = 480
            self.horizontalFov = 58.5
            self.verticalFov = 46.6
            self.maxBitDepth = 2047 # 11-bit
            self.minDepth = 400; # ?
            self.maxDepth = 5000; # ??


class KcVertSphere(object):

    def __init__(self):
        self._Displacement = 500
        self._BaselineLength = 180
        self._SphericalAngle = 3.142
        self._Maximum = 1000


class KcVert(object):

    def __init__(self):
        self.co = (0,0,0)
        self.uv = (0,0)
        self.n = (0,0,0)
        self.col = (0,0,0)
        self.depth = 0


# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ 
# REFERENCES

'''
// Links
// http://www.imaginativeuniversal.com/blog/2014/03/05/quick-reference-kinect-1-vs-kinect-2/
// https://smeenk.com/kinect-field-of-view-comparison/
// https://stackoverflow.com/questions/17832238/kinect-intrinsic-parameters-from-field-of-view
// https://rosindustrial.org/news/2016/1/13/3d-camera-survey
// https://opdhsblobprod03.blob.core.windows.net/contents/503db294612a42b3b95420aaabac44cc/77342d6514e7dbbcf477614ed3a7acda?sv=2015-04-05&sr=b&sig=Jn1XYV3R6%2Brh309dHaPO3BqCx5vtp1A%2BkCs%2F%2BTjhZlI%3D&st=2019-05-16T15%3A06%3A45Z&se=2019-05-17T15%3A16%3A45Z&sp=r
// https://forums.structure.io/t/structure-sensors-angle-of-view/486
// http://xtionprolive.com/primesense-carmine-1.09
// http://www.i3du.gr/pdf/primesense.pdf
// https://structure.io/structure-core/specs
// https://www.intel.com/content/www/us/en/support/articles/000030385/emerging-technologies/intel-realsense-technology.html


// Original OpenNI reference

OniStatus VideoStream::convertDepthToWorldCoordinates(float depthX, float depthY, float depthZ, float* pWorldX, float* pWorldY, float* pWorldZ)
{
  if (m_pSensorInfo->sensorType != ONI_SENSOR_DEPTH)
  {
    m_errorLogger.Append("convertDepthToWorldCoordinates: Stream is not from DEPTH\n");
    return ONI_STATUS_NOT_SUPPORTED;
  }

  float normalizedX = depthX / m_worldConvertCache.resolutionX - .5f;
  float normalizedY = .5f - depthY / m_worldConvertCache.resolutionY;

  *pWorldX = normalizedX * depthZ * m_worldConvertCache.xzFactor;
  *pWorldY = normalizedY * depthZ * m_worldConvertCache.yzFactor;
  *pWorldZ = depthZ;
  return ONI_STATUS_OK;
}

OniStatus VideoStream::convertWorldToDepthCoordinates(float worldX, float worldY, float worldZ, float* pDepthX, float* pDepthY, float* pDepthZ)
{
  if (m_pSensorInfo->sensorType != ONI_SENSOR_DEPTH)
  {
    m_errorLogger.Append("convertWorldToDepthCoordinates: Stream is not from DEPTH\n");
    return ONI_STATUS_NOT_SUPPORTED;
  }

  *pDepthX = m_worldConvertCache.coeffX * worldX / worldZ + m_worldConvertCache.halfResX;
  *pDepthY = m_worldConvertCache.halfResY - m_worldConvertCache.coeffY * worldY / worldZ;
  *pDepthZ = worldZ;
  return ONI_STATUS_OK;
}

void VideoStream::refreshWorldConversionCache()
{
  if (m_pSensorInfo->sensorType != ONI_SENSOR_DEPTH)
  {
    return;
  }

  OniVideoMode videoMode;
  int size = sizeof(videoMode);
  getProperty(ONI_STREAM_PROPERTY_VIDEO_MODE, &videoMode, &size);

  size = sizeof(float);
  float horizontalFov;
  float verticalFov;
  getProperty(ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &horizontalFov, &size);
  getProperty(ONI_STREAM_PROPERTY_VERTICAL_FOV, &verticalFov, &size);

  m_worldConvertCache.xzFactor = tan(horizontalFov / 2) * 2;
  m_worldConvertCache.yzFactor = tan(verticalFov / 2) * 2;
  m_worldConvertCache.resolutionX = videoMode.resolutionX;
  m_worldConvertCache.resolutionY = videoMode.resolutionY;
  m_worldConvertCache.halfResX = m_worldConvertCache.resolutionX / 2;
  m_worldConvertCache.halfResY = m_worldConvertCache.resolutionY / 2;
  m_worldConvertCache.coeffX = m_worldConvertCache.resolutionX / m_worldConvertCache.xzFactor;
  m_worldConvertCache.coeffY = m_worldConvertCache.resolutionY / m_worldConvertCache.yzFactor;
}

struct WorldConversionCache
{
  float xzFactor;
  float yzFactor;
  float coeffX;
  float coeffY;
  int resolutionX;
  int resolutionY;
  int halfResX;
  int halfResY;
} m_worldConvertCache;

# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ 

// vert sphere reference

class VertSphere {
  
  PImage tex_rgb;
  PImage tex_depth;
  float normLineLength = 20;
  int detail = 10;
  ArrayList<Vert> verts;
  boolean drawEndLines = false;
 
  float _Displacement = 500;
  float _BaselineLength = 180;
  float _SphericalAngle = 3.142;
  float _Maximum = 1000;
  
  VertSphere(PImage _rgb, PImage _depth) {
    init(_rgb, _depth);
  }
  
  VertSphere(PImage _rgb, PImage _depth, int _detail) {
    detail = _detail;
    init(_rgb, _depth);
  }
  
  void init(PImage _rgb, PImage _depth) {
    tex_rgb = _rgb;
    tex_depth = _depth;
    tex_rgb.loadPixels();
    tex_depth.loadPixels();
    verts = initVerts(detail);
  }
  
  ArrayList<Vert> initVerts(int detail) {
    ArrayList<Vert> returns = new ArrayList<Vert>();
    for (int m = 0; m < detail; m++) {
      for (int n = 0; n < detail; n++) {
        float x = sin(PI * m/detail) * cos(2 * PI * n/detail);
        float z = sin(PI * m/detail) * sin(2 * PI * n/detail);
        float y = cos(PI * m/detail); // up
          
        Vert v = new Vert(new PVector(x, y, z));
        v.col = getPixelFromUv(tex_rgb, v.uv);
        v.depth = getPixelFromUv(tex_depth, v.uv);
        v.co = reprojectEqr(v);
        returns.add(v);
      }
    }

    return returns;
  }
  
  void draw() {
    stroke(255);
    strokeWeight(2);
    draw_points();
  }

  void draw_points() {
    for (int i = 0; i < verts.size(); i++) {
      Vert v = verts.get(i);
      
      stroke(v.col);
      strokeWeight(10);
      point(v.co.x + v.n.x, v.co.y + v.n.y, v.co.z + v.n.z);
      
      if (drawEndLines) {
        PVector end = v.co.copy().add(v.n.copy().mult(normLineLength));
        strokeWeight(2);
        line(v.co.x, v.co.y, v.co.z, end.x, end.y, end.z);
      }
    }
  }

  color getPixelFromUv(PImage img, PVector uv) {   
    int x = int(uv.x * img.width);
    int y = int(uv.y * img.height);
    int loc = x + y * img.width;
    loc = constrain(loc, 0, img.pixels.length - 1);
    return img.pixels[loc];
  }
  
  float getDepthSpherical(float d) {
      return asin(_BaselineLength * sin(_SphericalAngle)) / asin(d);
  }
          
  PVector reprojectEqr(Vert v) {
    PVector returns = v.n.copy().mult(constrain(getDepthSpherical(red(v.depth)/255.0), -_Maximum, 0) * _Displacement);
    return new PVector(returns.x, returns.y, returns.z);
  }

}


class Vert {
  
  PVector co;
  PVector uv;
  PVector n;
  color col;
  color depth;
  
  Vert() {
    co = new PVector(0,0,0);
    col = color(0);
    depth = color(0);
    n = co.copy().normalize();
    uv = getUv(co);
  }
  
  Vert(PVector _co) {
    co = _co;
    col = color(0);
    depth = color(0);
    n = co.copy().normalize();
    uv = getUv(co);
  }
  
  Vert(PVector _co, color _col) {
    co = _co;
    col = _col;
    depth = color(0);
    n = co.copy().normalize();
    uv = getUv(co);
  }
  
  Vert(PVector _co, PVector _uv) {
    co = _co;
    col = color(0);
    depth = color(0);
    n = co.copy().normalize();
    uv = _uv;
  }

  Vert(PVector _co, PVector _uv, color _col) {
    co = _co;
    col = _col;
    depth = color(0);
    n = co.copy().normalize();
    uv = _uv;
  }
  
  Vert(float x, float y, float z) {
    co = new PVector(x, y, z);
    col = color(0);
    depth = color(0);
    n = co.copy().normalize();
    uv = getUv(co);
  }
  
  Vert(float x, float y, float z, color _col) {
    co = new PVector(x, y, z);
    col = _col;
    depth = color(0);
    n = co.copy().normalize();
    uv = getUv(co);
  }
  
  Vert(float x, float y, float z, float u, float v) {
    co = new PVector(x, y, z);
    col = color(0);
    depth = color(0);
    n = co.copy().normalize();
    uv = new PVector(u, v);
  }

  Vert(float x, float y, float z, float u, float v, color _col) {
    co = new PVector(x, y, z);
    col = _col;
    depth = color(0);
    n = co.copy().normalize();
    uv = new PVector(u, v);
  }
  
  PVector getUv(PVector p) {
    p = new PVector(p.x, p.y, p.z).normalize();
    float u = 0.5 + (atan2(p.x, p.z) / (2 * PI)); 
    float v = 0.5 - (asin(p.y) / PI);
    return new PVector(1.0 - u, v);
  }
  
  PVector getUvPShape(PVector p) {
    p = new PVector(p.z, 1.0 - p.y, p.x).normalize();
    float u = 0.5 + (atan2(p.x, p.z) / (2 * PI)); 
    float v = 0.5 - (asin(p.y) / PI);
    return new PVector(0.5 + u, v);
  }
  
  PVector getXyz(float u, float v) {
    float theta = u * 2.0 * PI;
    float phi = (v - 0.5) * PI;
    float c = cos(phi);
    return new PVector(c * cos(theta), sin(phi), c * sin(theta));
  }
  
}
'''
