/*\
 *

  ESP32-Tesseract

  A TFT/sprite demo brought to you by tobozo
  Loop concept taken from https://husarnet.com

  MIT License

  Copyright (c) 2020 tobozo

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  -----------------------------------------------------------------------------

 *
\*/


#include <ESP32-Chimera-Core.h> // https://github.com/tobozo/ESP32-Chimera-Core or Arduino Library Manager

#include "lookup_tables.h"
#include "heart_anim.h"

#define tft M5.Lcd
TFT_eSprite sprite = TFT_eSprite( &tft );
TFT_eSprite heartSprite = TFT_eSprite( &tft );

typedef std::vector<std::vector<float>> PointsArray;
typedef std::array<float,4> Coords;
typedef std::array<uint8_t,4> ByteCoords;

typedef std::vector<std::array<int16_t,2>> LinesIndexesArray;
typedef std::vector<int16_t> PointsIndexesArray;
typedef std::array<int16_t,4> Coords4D;


struct FrameSrc {
  const uint8_t *jpeg;
  size_t   jpeg_len;
};

struct FrameDimensions {
  uint16_t width;
  uint16_t height;
};

struct Animation {
  FrameSrc *frames;
  FrameDimensions dimensions;
  uint16_t width() {
    return dimensions.width;
  }
  uint16_t height() {
    return dimensions.height;
  }
  FrameSrc frame(uint16_t framenum) {
    return frames[framenum];
  }
};

#define framesCount 60
FrameDimensions heart_dimensions = { 48, 55 };

FrameSrc heart_frames[framesCount] = {
  { heart_00_jpg, heart_00_jpg_len},
  { heart_01_jpg, heart_01_jpg_len},
  { heart_02_jpg, heart_02_jpg_len},
  { heart_03_jpg, heart_03_jpg_len},
  { heart_04_jpg, heart_04_jpg_len},
  { heart_05_jpg, heart_05_jpg_len},
  { heart_06_jpg, heart_06_jpg_len},
  { heart_07_jpg, heart_07_jpg_len},
  { heart_08_jpg, heart_08_jpg_len},
  { heart_09_jpg, heart_09_jpg_len},
  { heart_10_jpg, heart_10_jpg_len},
  { heart_11_jpg, heart_11_jpg_len},
  { heart_12_jpg, heart_12_jpg_len},
  { heart_13_jpg, heart_13_jpg_len},
  { heart_14_jpg, heart_14_jpg_len},
  { heart_15_jpg, heart_15_jpg_len},
  { heart_16_jpg, heart_16_jpg_len},
  { heart_17_jpg, heart_17_jpg_len},
  { heart_18_jpg, heart_18_jpg_len},
  { heart_19_jpg, heart_19_jpg_len},
  { heart_20_jpg, heart_20_jpg_len},
  { heart_21_jpg, heart_21_jpg_len},
  { heart_22_jpg, heart_22_jpg_len},
  { heart_23_jpg, heart_23_jpg_len},
  { heart_24_jpg, heart_24_jpg_len},
  { heart_25_jpg, heart_25_jpg_len},
  { heart_26_jpg, heart_26_jpg_len},
  { heart_27_jpg, heart_27_jpg_len},
  { heart_28_jpg, heart_28_jpg_len},
  { heart_29_jpg, heart_29_jpg_len},
  { heart_30_jpg, heart_30_jpg_len},
  { heart_31_jpg, heart_31_jpg_len},
  { heart_32_jpg, heart_32_jpg_len},
  { heart_33_jpg, heart_33_jpg_len},
  { heart_34_jpg, heart_34_jpg_len},
  { heart_35_jpg, heart_35_jpg_len},
  { heart_36_jpg, heart_36_jpg_len},
  { heart_37_jpg, heart_37_jpg_len},
  { heart_38_jpg, heart_38_jpg_len},
  { heart_39_jpg, heart_39_jpg_len},
  { heart_40_jpg, heart_40_jpg_len},
  { heart_41_jpg, heart_41_jpg_len},
  { heart_42_jpg, heart_42_jpg_len},
  { heart_43_jpg, heart_43_jpg_len},
  { heart_44_jpg, heart_44_jpg_len},
  { heart_45_jpg, heart_45_jpg_len},
  { heart_46_jpg, heart_46_jpg_len},
  { heart_47_jpg, heart_47_jpg_len},
  { heart_48_jpg, heart_48_jpg_len},
  { heart_49_jpg, heart_49_jpg_len},
  { heart_50_jpg, heart_50_jpg_len},
  { heart_51_jpg, heart_51_jpg_len},
  { heart_52_jpg, heart_52_jpg_len},
  { heart_53_jpg, heart_53_jpg_len},
  { heart_54_jpg, heart_54_jpg_len},
  { heart_55_jpg, heart_55_jpg_len},
  { heart_56_jpg, heart_56_jpg_len},
  { heart_57_jpg, heart_57_jpg_len},
  { heart_58_jpg, heart_58_jpg_len},
  { heart_59_jpg, heart_59_jpg_len}
};

Animation animation = {
  heart_frames,
  heart_dimensions
};

int aframe = 0; // animation frame number

uint16_t screenWidth = 238;
uint16_t screenHeight = 238;
uint16_t spritePosX;
uint16_t spritePosY;

float fov = .75; // field of view for perspective
float baseHScale = 1.25;
float baseVScale = 1.25;
float depthScale = (screenWidth*baseHScale)/50;
float sphereMass = (screenWidth / 50*baseHScale);

float fps = 0;
int fpscount;
unsigned long lastfps = millis();

int16_t zSortedPoints4D[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
float referenceAngle = 0.0;
float QuarterPI = PI / 4;
float FourPi =  4 * PI;
float scaletobyte = 1.0 / 256.0; // cheap minifloat
bool linesProcessed = false;

Coords tmpCoords;

PointsArray rot4D = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
PointsArray rotXY;
PointsArray rotXZ;
PointsArray rotYZ;
PointsArray rotXW;
PointsArray rotZW;
PointsArray paXY;
PointsArray paZW;
PointsArray paID;
// TODO: use less of those
PointsArray projection1;
PointsArray projection2;
PointsArray projection3;
PointsArray projection4;

Coords4D cache4D[16];
LinesIndexesArray lines4D;
PointsIndexesArray points4D;
RGBColor colorstart;
RGBColor colorend;

PointsArray rot3d = {
  { 1, 0, 0 },
  { 0, (float)romcos(QuarterPI), (float)-romsin(QuarterPI) },
  { 0, (float)romsin(QuarterPI), (float)romcos(QuarterPI) }
};


bool pointIsIndexed( int16_t point ) {
  for(byte i=0;i<points4D.size();i++) {
    if( point == points4D[i] ) {
      return true;
    }
  }
  return false;
}


void pointPush( int16_t point ) {
  if( ! pointIsIndexed( point ) ) {
    points4D.push_back( point );
  }
}

bool lineIsIndexed( std::array<int16_t,2> line ) {
  for(byte i=0;i<lines4D.size();i++) {
    if( (line[0] == lines4D[i][0] && line[1] == lines4D[i][1])
     || (line[1] == lines4D[i][0] && line[0] == lines4D[i][1])
    ) {
      return true;
    }
  }
  return false;
}

void linePush( std::array<int16_t,2> line ) {
  if( !lineIsIndexed( line ) ) {
    lines4D.push_back( line );
  }
}

/* sort xyz by z depth */
void zSortPoints() {
  bool swapped;
  int16_t temp;
  float zdepth, nextzdepth;
  do {
    swapped = false;
    for(int16_t i=0; i<15; i++ ) {
      zdepth     = cache4D[ zSortedPoints4D[i]   ][2];
      nextzdepth = cache4D[ zSortedPoints4D[i+1] ][2];
      if ( zdepth > nextzdepth ) {
        temp = zSortedPoints4D[i];
        zSortedPoints4D[i] = zSortedPoints4D[i + 1];
        zSortedPoints4D[i + 1] = temp;
        swapped = true;
      }
    }
  } while (swapped);
}

void setCoordsCache( ByteCoords cc, Coords &pout ) {
  byte x = cc[0], y=cc[1], z=cc[2], w=cc[3];
  int16_t cacheID = x + y*2 + z*4 + w*8;
  cache4D[cacheID] = { (int16_t)pout[0], (int16_t)pout[1], (int16_t)pout[2], (int16_t)pout[3] };
}

bool isInCoordsCache( ByteCoords cc ) {
  byte x = cc[0], y=cc[1], z=cc[2], w=cc[3];
  int16_t cacheID = x + y*2 + z*4 + w*8;
  if( cache4D[cacheID][0] + cache4D[cacheID][1] + cache4D[cacheID][2] + cache4D[cacheID][3] == 0 ) {
    return false;
  }
  return true;
}

void clearCoordsCache() {
  for( byte  i=0;i<16;i++ ) {
    cache4D[i] = {0,0,0,0};
  }
}


void multiplyCoords( PointsArray &A, PointsArray &B, PointsArray &C) {
  C.clear();
  for (byte i = 0; i < A.size(); i++) {
    std::vector<float> mm;
    for (byte j = 0; j < B[0].size(); j++) {
      float sum = 0;
      for (byte k = 0; k < A[0].size(); k++) {
        sum += A[i][k] * B[k][j];
      }
      mm.push_back( sum );
    }
    C.push_back( mm );
  }
}


void multiplyCoords( PointsArray &A, Coords &B, PointsArray &C) {
  PointsArray CoordsToPointsArray(4);
  for(byte i=0;i<B.size();i++) {
    CoordsToPointsArray[i].push_back(B[i]);
  }
  multiplyCoords( A, CoordsToPointsArray, C );
}


void transformPoint(ByteCoords p0) {

  if( isInCoordsCache( p0 ) ) {
    return;
  }

  for (int i = 0; i < p0.size(); i++) {
    tmpCoords[i] = (p0[i] - 0.5);
  }
  // use the sin/cos lookup table
  float romcosrefangle = romcos(referenceAngle);
  float romsinrefangle = romsin(referenceAngle);

  rotXY = {
    { romcosrefangle, -romsinrefangle, 0, 0},
    { romsinrefangle, romcosrefangle, 0, 0},
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };

  rotXZ = {
    { romcosrefangle, 0, -romsinrefangle, 0 },
    { romsinrefangle, 0, romcosrefangle, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };

  rotYZ = {
    { 1, romcosrefangle, -romsinrefangle, 0 },
    { 0, romsinrefangle, romcosrefangle, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  };

  rotXW = {
    { romcosrefangle, 0, 0, -romsinrefangle },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { romsinrefangle, 0, 0, romcosrefangle }
  };

  rotZW = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, romcosrefangle, -romsinrefangle },
    { 0, 0, romsinrefangle, romcosrefangle }
  };


  multiplyCoords( rotXY, tmpCoords, paXY );
  multiplyCoords( rotZW, paXY, paZW );
  multiplyCoords( rot4D, paZW, projection2 );

  /*
  //multiplyCoords( rotXY, tmpCoords, paXY );
  multiplyCoords( rotZW, tmpCoords, paZW );
  multiplyCoords( rot4D, paZW, projection2 );
  */
  /*
  multiplyCoords( rotXY, tmpCoords, paXY );
  multiplyCoords( rotZW, paXY, paZW );
  multiplyCoords( rot4D, paZW, projection2 );
  */
  float distance = 3;
  float w = 1 / (distance - projection2[3][0]);

  projection1 = {
    { w, 0, 0, 0 },
    { 0, w, 0, 0 },
    { 0, 0, w, 0 }
  };

  multiplyCoords( projection1, projection2, projection3); // project in 3d
  multiplyCoords( rot3d, projection3, projection4); // apply 3D rotation
  // add some depth of field
  projection4[0][0] += fov*projection4[0][0]*projection4[2][0];
  projection4[1][0] += fov*projection4[1][0]*projection4[2][0];

  Coords pout = {((projection4[0][0] * screenWidth ) * baseHScale)+screenWidth/2, ((projection4[1][0] * screenHeight) * baseVScale)+screenHeight/2, (projection4[2][0]*255)+128 };
  setCoordsCache( p0, pout );
}


uint16_t luminance(uint16_t color, uint8_t luminance) {
  // Extract rgb colours and stretch range to 0 - 255
  uint16_t r = (color & 0xF800) >> 8; r |= (r >> 5);
  uint16_t g = (color & 0x07E0) >> 3; g |= (g >> 6);
  uint16_t b = (color & 0x001F) << 3; b |= (b >> 5);

  b = ((b * (uint16_t)luminance + 255) >> 8) & 0x00F8;
  g = ((g * (uint16_t)luminance + 255) >> 8) & 0x00FC;
  r = ((r * (uint16_t)luminance + 255) >> 8) & 0x00F8;

  return (r << 8) | (g << 3) | (b >> 3);
}


void drawSphere( int16_t x, int16_t y, int16_t radius, int16_t color ) {
  int16_t _radius = radius;
  //int halfradius = radius / 2;
  sprite.drawCircle( x, y, radius+1, TFT_BLACK);
  sprite.fillCircle( x, y, radius, color);
  while( _radius > 0 ) {
    int gap = (radius - _radius)/2;
    byte lumval = map( _radius, 0, radius, 255, 64 );
    uint16_t translatedColor = luminance( color, lumval );
    sprite.fillCircle( x+gap, y-gap, _radius, translatedColor);
    _radius--;
  }
}


void calcLinesFromPoint( uint8_t x, uint8_t y, uint8_t z, uint8_t w ) {

  int16_t _2y = y*2;
  int16_t _4z = z*4;
  int16_t _8w = w*8;

  int16_t
    pointIndex =   x +     _2y +     _4z +     _8w,
    line1Index = 1-x +     _2y +     _4z +     _8w,
    line2Index =   x + (1-y)*2 +     _4z +     _8w,
    line3Index =   x +     _2y + (1-z)*4 +     _8w,
    line4Index =   x +     _2y +     _4z + (1-w)*8
  ;
  if( !linesProcessed ) {
    pointPush( pointIndex );
    linePush( {pointIndex, line1Index} );
    linePush( {pointIndex, line2Index} );
    linePush( {pointIndex, line3Index} );
    linePush( {pointIndex, line4Index} );
  }
  transformPoint({x, y, z, w});
}


void drawPoints() {

  zSortPoints();

  for(int8_t i=0;i<16;i++) {

    int16_t srtindex = zSortedPoints4D[i];

    for(int8_t j=0;j<32;j++) {

      int16_t srcindex = lines4D[j][0];
      int16_t dstindex = lines4D[j][1];

      if( srcindex == srtindex || dstindex == srtindex ) {
        //TODO: don't process a line twice
        byte valstart = cache4D[srcindex][2];
        byte valend   = cache4D[dstindex][2];

        colorstart = { valstart, valstart, valstart };
        colorend   = { valend,   valend,   valend };

        sprite.drawGradientLine(
          cache4D[srcindex][0], cache4D[srcindex][1],
          cache4D[dstindex][0], cache4D[dstindex][1],
          colorstart,
          colorend
        );
      }
    }

    if( i==7 ) {
      sprite.pushImage(
        sprite.width()/2-(animation.width()/2),
        sprite.height()/2-(animation.height()/2),
        animation.width(), animation.height(),
        (uint16_t*)heartSprite.frameBuffer(0),
        TFT_BLACK
      );
    }

    float r = sphereMass + cache4D[srtindex][2]*scaletobyte*depthScale;
    uint16_t sphereColor = tft.color565( cache4D[srtindex][2], cache4D[srtindex][2], cache4D[srtindex][2] );

    drawSphere( cache4D[srtindex][0], cache4D[srtindex][1], r/*cache4D[srtindex][2]*/, sphereColor );
  }
}




void drawTesseract() {
  sprite.fillSprite( TFT_BLACK );

  for (uint8_t x=0; x <= 1; x ++) {
    for (uint8_t y=0; y <= 1; y ++) {
      for (uint8_t z=0; z <= 1; z ++) {
        for (uint8_t w=0; w <= 1; w ++) {
          calcLinesFromPoint( x, y, z, w );
        }
      }
    }
  }

  drawPoints();

  clearCoordsCache();


  unsigned long fpsdelay = millis() - lastfps;
  if( fpsdelay >= 1000 ) {
    fps = float(fpscount*1000)/float(fpsdelay);
    lastfps = millis();
    fpscount = 0;
  }

  sprite.setCursor( 0, 0 );
  sprite.printf("%.2f", fps);

  sprite.pushSprite( spritePosX, spritePosY );

  fpscount++;

  aframe++;
  aframe = aframe%framesCount;
  int currentframe = framesCount-(aframe+1);
  Serial.println( currentframe );

  heartSprite.drawJpg( animation.frame(currentframe).jpeg, animation.frame(currentframe).jpeg_len, 0, 0 );

  if( !linesProcessed ) {
    linesProcessed = true;
  }

}


void setup() {
  M5.begin();

  sprite.setColorDepth( 16 ); // set this to 8 when screenWidth*screenHeight > 238*238
  sprite.createSprite( screenWidth, screenHeight );
  sprite.setSwapBytes( true ); // for receiving color data from another heartSprite
  spritePosX = tft.width()/2 - screenWidth/2;
  spritePosY = tft.height()/2 - screenHeight/2;

  heartSprite.setColorDepth( 16 );
  heartSprite.createSprite( animation.width(), animation.height() );
  heartSprite.drawJpg( animation.frame(0).jpeg, animation.frame(0).jpeg_len, 0, 0 );

  Serial.printf("Loaded animation %d*%d at [%d:%d]", animation.width(), animation.height(), tft.width()/2-(animation.width()/2), tft.height()/2-(animation.height()/2) );

  aframe++;

}

void loop() {
  referenceAngle = millis() / 1250.0;
  referenceAngle = fmod( referenceAngle, FourPi );
  drawTesseract();
}

