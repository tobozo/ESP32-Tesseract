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
#define tft M5.Lcd

#include "AmigaRulez.h"
#include "lookup_tables.h"
#include "heart_anim.h"
#include "gfx.h"

// uncommenting this will capture images and save them on the SD, very slow !
// #define CAPTURE_MODE

enum AnimationTypes {
  ANIMATION_AMIGA,
  ANIMATION_HEART
};

// set this to display the heart animation or the amiga ball
AnimationTypes AnimationType;// = ANIMATION_AMIGA;

int aframe = 0; // animation frame number

#if defined(ARDUINO_TTGO_T1)
  // sweet ~50 fps on 128x128
  uint16_t screenWidth = 128;
  uint16_t screenHeight = 128;
#else
  // honest ~30 fps on 238x238
  uint16_t screenWidth = 238;
  uint16_t screenHeight = 238;
#endif


uint16_t spritePosX;
uint16_t spritePosY;
uint16_t corePosX;
uint16_t corePosY;

float fov = .75; // field of view for perspective
float baseHScale = 1.25;
float baseVScale = 1.25;
float depthScale = (screenWidth*baseHScale)/50;
float sphereMass = (screenWidth / 50*baseHScale);

static float fps = 0;
static int fpscount;
static unsigned long lastfps = millis();
static int captured = 0;
static unsigned long timetraveller = millis();

static int16_t zSortedPoints4D[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
static int16_t zSortedPoints4Dbuff[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

static float referenceAngle = 0.0;
static float QuarterPI = PI / 4.0;
static float TwoPi =  2.0* PI;
static float FourPi =  4.0 * PI;
static float scaletobyte = 1.0 / 256.0; // cheap minifloat
static bool linesProcessed = false;

static bool drawing = false;
static bool needsdrawing = false;
static bool dostrobe = true;

static Coords tmpCoords;

static PointsArray rot4D = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
static PointsArray rotXY;
static PointsArray rotXZ;
static PointsArray rotYZ;
static PointsArray rotXW;
static PointsArray rotZW;
static PointsArray paXY;
static PointsArray paZW;
static PointsArray paID;

// TODO: use less of those
static PointsArray projection1;
static PointsArray projection2;
static PointsArray projection3;
static PointsArray projection4;

static Coords4D cache4D[16];
static Coords4D cache4Dbuff[16];

static LinesIndexesArray lines4DBuff;
static LinesIndexesArray lines4D;

static PointsIndexesArray points4D;

static RGBColor colorstart;
static RGBColor colorend;

static PointsArray rot3d = {
  { 1, 0, 0 },
  { 0, (float)romcos(QuarterPI), (float)-romsin(QuarterPI) },
  { 0, (float)romsin(QuarterPI), (float)romcos(QuarterPI) }
};


static bool pointIsIndexed( int16_t point ) {
  for(byte i=0;i<points4D.size();i++) {
    if( point == points4D[i] ) {
      return true;
    }
  }
  return false;
}


static void pointPush( int16_t point ) {
  if( ! pointIsIndexed( point ) ) {
    points4D.push_back( point );
  }
}


static bool lineIsIndexed( std::array<int16_t,2> line ) {
  for(byte i=0;i<lines4D.size();i++) {
    if( (line[0] == lines4D[i][0] && line[1] == lines4D[i][1])
     || (line[1] == lines4D[i][0] && line[0] == lines4D[i][1])
    ) {
      return true;
    }
  }
  return false;
}


static void linePush( std::array<int16_t,2> line ) {
  if( !lineIsIndexed( line ) ) {
    lines4D.push_back( line );
  }
}


/* sort xyz by z depth */
static void zSortPoints() {
  bool swapped;
  int16_t temp;
  float zdepth, nextzdepth;
  do {
    swapped = false;
    for(int16_t i=0; i<15; i++ ) {
      zdepth     = cache4Dbuff[ zSortedPoints4Dbuff[i]   ][2];
      nextzdepth = cache4Dbuff[ zSortedPoints4Dbuff[i+1] ][2];
      if ( zdepth > nextzdepth ) {
        temp = zSortedPoints4Dbuff[i];
        zSortedPoints4Dbuff[i] = zSortedPoints4Dbuff[i + 1];
        zSortedPoints4Dbuff[i + 1] = temp;
        swapped = true;
      }
    }
  } while (swapped);
}


static void setCoordsCache( ByteCoords cc, Coords &pout ) {
  byte x = cc[0], y=cc[1], z=cc[2], w=cc[3];
  int16_t cacheID = x + y*2 + z*4 + w*8;
  cache4D[cacheID] = { (int16_t)pout[0], (int16_t)pout[1], (int16_t)pout[2], (int16_t)pout[3] };
}


static bool isInCoordsCache( ByteCoords cc ) {
  byte x = cc[0], y=cc[1], z=cc[2], w=cc[3];
  int16_t cacheID = x + y*2 + z*4 + w*8;
  if( cache4D[cacheID][0] + cache4D[cacheID][1] + cache4D[cacheID][2] + cache4D[cacheID][3] == 0 ) {
    return false;
  }
  return true;
}


static void clearCoordsCache() {
  for( byte  i=0;i<16;i++ ) {
    cache4D[i] = {0,0,0,0};
  }
}


static void multiplyCoords( PointsArray &A, PointsArray &B, PointsArray &C) {
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


static void multiplyCoords( PointsArray &A, Coords &B, PointsArray &C) {
  PointsArray CoordsToPointsArray(4);
  for(byte i=0;i<B.size();i++) {
    CoordsToPointsArray[i].push_back(B[i]);
  }
  multiplyCoords( A, CoordsToPointsArray, C );
}


static void transformPoint(ByteCoords p0) {

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

  multiplyCoords( projection1, projection2, projection3); // project 4d to 3d
  multiplyCoords( rot3d, projection3, projection4); // project 3d to 2d
  // add some depth of field
  projection4[0][0] += fov*projection4[0][0]*projection4[2][0];
  projection4[1][0] += fov*projection4[1][0]*projection4[2][0];

  Coords pout = {((projection4[0][0] * screenWidth ) * baseHScale)+screenWidth/2, ((projection4[1][0] * screenHeight) * baseVScale)+screenHeight/2, (projection4[2][0]*256)+127 };
  setCoordsCache( p0, pout );
}


static void calcLinesFromPoint( uint8_t x, uint8_t y, uint8_t z, uint8_t w ) {

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
    linePush( {pointIndex, line1Index} );
    linePush( {pointIndex, line2Index} );
    linePush( {pointIndex, line3Index} );
    linePush( {pointIndex, line4Index} );
    pointPush( pointIndex );
  }
  transformPoint({x, y, z, w});
}


static void drawPoints() {

  zSortPoints();

  for(int8_t i=0;i<16;i++) {

    int16_t srtindex = zSortedPoints4Dbuff[i];

    for(int8_t j=0;j<32;j++) {

      int16_t srcindex = lines4D[j][0];
      int16_t dstindex = lines4D[j][1];

      if( srcindex == srtindex || dstindex == srtindex ) {
        //TODO: don't process a line twice
        byte valstart = cache4Dbuff[srcindex][2];
        byte valend   = cache4Dbuff[dstindex][2];

        colorstart = { byte(255-valstart/2), byte(192-valstart/2), byte(203-valstart/2) };
        colorend   = { byte(255-valend/2),   byte(192-valend/2),   byte(203-valend/2) };

        float r0 = sphereMass + cache4Dbuff[srtindex][2]*scaletobyte*depthScale;
        float r1 = sphereMass + cache4Dbuff[dstindex][2]*scaletobyte*depthScale;

        drawPartialGradientLineAntialiased(
          &sprite,
          cache4Dbuff[srcindex][0], cache4Dbuff[srcindex][1],
          cache4Dbuff[dstindex][0], cache4Dbuff[dstindex][1],
          colorstart,
          colorend,
          r0,
          r1
        );
      }
    }


    if( i==7 ) {
      sprite.pushImage(
        corePosX,
        corePosY,
        animation.width(),
        animation.height(),
        (uint16_t*)coreSprite.frameBuffer(0),
        TFT_BLACK
      );
    }

    float r = sphereMass + cache4Dbuff[srtindex][2]*scaletobyte*depthScale;
    float c = map( cache4Dbuff[srtindex][2], 0, 255, 192, 255 );
    uint16_t sphereColor = tft.color565( c,c,c );
    Coords3D sphereCoords = { cache4Dbuff[srtindex][0], cache4Dbuff[srtindex][1], cache4Dbuff[srtindex][2]};
    drawCache3dSphere( &sprite, sphereCoords, r*2., sphereColor );
  }
}


static void drawTesseract() {

  unsigned long time_before_draw = millis();

  if( dostrobe ) {
    spriteFadeOut( &sprite, 192 );
  } else {
    sprite.fillSprite( TFT_BLACK );
  }

  drawPoints();

  sprite.setCursor( 0, 0 );
  sprite.printf("fps: %2.2f\n", fps);
  sprite.printf("balls: %d", sphereCache.size()+1);

  sprite.pushSprite( spritePosX, spritePosY );

  unsigned long time_before_capture = millis();

  #ifdef CAPTURE_MODE
    char fileName[32];
    sprintf( fileName, "/jpg/tesseract-%03d.bmp", captured );
    M5.ScreenShot.snapBMP( fileName );
    // sprintf( fileName, "/jpg/tesseract-%03d.jpg", captured );
    // M5.ScreenShot.snapJPG( fileName );
    vTaskDelay(1);
  #endif

  unsigned long time_to_capture = millis() - time_before_capture;

  captured++;
  fpscount++;

  //aframe++;
  aframe = aframe%framesCount;
  int currentframe = framesCount-(aframe+1);
  //Serial.println( currentframe );

  switch( AnimationType ) {
    case ANIMATION_AMIGA:
      AmigaBall.Phase = fmod( AmigaBall.Phase + ( AmigaBall.phase4Rad - AmigaBall.PhaseVelocity ), AmigaBall.phase4Rad );
      //AmigaBall.Phase = fmod( Phase + AmigaBall.PhaseVelocity, AmigaBall.phase4Rad );
      AmigaBall.drawBall( AmigaBall.Phase, coreSprite.width()/2, coreSprite.height()/2, coreSprite.width()/2, coreSprite.height()/2);
    break;
    case ANIMATION_HEART:
      coreSprite.drawJpg( animation.frame(currentframe).jpeg, animation.frame(currentframe).jpeg_len, 0, 0 );
    break;
  }

  if( !linesProcessed ) {
    linesProcessed = true;
  }

  unsigned long time_to_draw = millis() - time_before_draw;
  unsigned long real_time_to_draw = time_to_draw - time_to_capture;
  timetraveller += real_time_to_draw;

  unsigned long fpsdelay = timetraveller - lastfps;
  if( fpsdelay >= 1000 ) {
    fps = float(fpscount*1000)/float(fpsdelay);
    lastfps = timetraveller;
    fpscount = 0;
  }

}


static void doCalcCoords() {
  referenceAngle = timetraveller/*millis()*/ / /*2500.0*/ 1250.0 /*625.0*/;
  referenceAngle = fmod( referenceAngle, FourPi );
  aframe = map( referenceAngle*1000, 0, TwoPi*1000, 0, framesCount-1);

  clearCoordsCache();

  for (uint8_t x=0; x <= 1; x ++) {
    for (uint8_t y=0; y <= 1; y ++) {
      for (uint8_t z=0; z <= 1; z ++) {
        for (uint8_t w=0; w <= 1; w ++) {
          calcLinesFromPoint( x, y, z, w );
        }
      }
    }
  }
  // wait for the current drawing to finish
  while( drawing ) vTaskDelay( 1 );
  // transfer coords to buffer
  memcpy( cache4Dbuff, cache4D, sizeof(Coords4D)*16 );
  memcpy( zSortedPoints4Dbuff, zSortedPoints4D, sizeof(int16_t)*16 );
  // send draw signal
  needsdrawing = true;
}


static void calcCoordsTask( void * param ) {
  while( 1 ) {
    // prevent a draw to begin as data will
    doCalcCoords();
    vTaskDelay( 1 );
  }
  vTaskDelete( NULL );
}


static void mainTask( void * param ) {

  clearCoordsCache();

  xTaskCreatePinnedToCore( calcCoordsTask, "calcCoordsTask", 2048, NULL, 0, NULL, 1 ); /* last = Task Core */

  while(1) {

    if( needsdrawing ) {
      needsdrawing = false;
      drawing = true;
      drawTesseract();
      drawing = false;
    }
    vTaskDelay(1);
  }
  vTaskDelete( NULL );
}



void setup() {
  M5.begin();
  tft.clear();

  #ifdef CAPTURE_MODE
    M5.ScreenShot.init( &tft, M5STACK_SD );
    M5.ScreenShot.begin();
  #endif

  sprite.setAttribute( PSRAM_ENABLE, false );
  coreSprite.setAttribute( PSRAM_ENABLE, false );

  #if defined(ARDUINO_LOLIN_D32_PRO)
    // tft.setRotation(3);
  #elif defined(ARDUINO_ESP32_DEV) // wrover kit
    tft.setRotation(0);
  #elif defined(ARDUINO_DDUINO32_XS)
    // tft.setRotation(0);
  #elif defined(ARDUINO_TTGO_T1)
    tft.setRotation(0);
  #elif defined(ARDUINO_ODROID_ESP32)
    // tft.setRotation(0);
  #else // m5stack classic/fire
    // tft.setRotation(1);
  #endif

  sprite.setColorDepth( 16 ); // set this to 8 when screenWidth*screenHeight > 238*238, since pram is disabled for sprites in this demo
  sprite.createSprite( screenWidth, screenHeight );
  sprite.setSwapBytes( true ); // for receiving color data from another coreSprite
  spritePosX = tft.width()/2 - screenWidth/2;
  spritePosY = tft.height()/2 - screenHeight/2;
  sprite.fillSprite( TFT_BLACK );

  coreSprite.setColorDepth( 16 );
  coreSprite.createSprite( animation.width(), animation.height() );
  coreSprite.fillSprite( TFT_BLACK );

  corePosX = sprite.width()/2-(animation.width()/2);
  corePosY = sprite.height()/2-(animation.height()/2);

  amigaBallConfig.Width = coreSprite.width();
  amigaBallConfig.Height = coreSprite.height();
  amigaBallConfig.ScaleRatio = 5;
  amigaBallConfig.sprite = &coreSprite;
  AmigaBall.init( amigaBallConfig );

  sprite.pushSprite( 0, 0 );

  Serial.printf("Loaded animation %d*%d at [%d:%d]\n", animation.width(), animation.height(), tft.width()/2-(animation.width()/2), tft.height()/2-(animation.height()/2) );

  aframe++;
  timetraveller = 0;

  dostrobe = false;
  AnimationType = ANIMATION_AMIGA;

  #ifndef CAPTURE_MODE
    xTaskCreatePinnedToCore( mainTask, "mainTask", 3072, NULL, 32, NULL, 0 ); /* last = Task Core */
  #endif

}


void loop() {
  #ifdef CAPTURE_MODE
    // capturing display from task doesn't work well
    // so this is done from the main loop instead
    doCalcCoords();
    drawTesseract();
  #else
    vTaskSuspend(NULL);
  #endif
}
