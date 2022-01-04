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
#define SDU_APP_NAME   "Tesseract" // app title for the sd-updater lobby screen
#define SDU_APP_PATH   "/Tesseract.bin"     // app binary file name on the SD Card (also displayed on the sd-updater lobby screen)
#define SDU_APP_AUTHOR "@tobozo"           // app binary author name for the sd-updater lobby screen
#include <M5StackUpdater.h>

#if defined(ARDUINO_TTGO_T1)
  // sweet ~50 fps on 128x128 (assuming SPI@27MHz)
  uint16_t screenWidth = 128;
  uint16_t screenHeight = 128;
#else
  // honest ~30 fps on 230x230 (assuming SPI@40MHz)
  uint16_t screenWidth = 220;
  uint16_t screenHeight = 220;
#endif

// uncommenting this will capture images and save them on the SD, very slow !
// #define CAPTURE_MODE
#if defined CAPTURE_MODE
  static bool ScreenShotEnable = true;
#else
  static bool ScreenShotEnable = false;
#endif


static LGFX &tft(M5.Lcd);
static LGFX_Sprite tmpsprite( &tft );
static LGFX_Sprite sprite( &tft );
static LGFX_Sprite coreSprite( &tft );

#include "tesseract.h"      // 5D logic
//#include "qoi_impl.h"       // qoi decoder
#include "gfx.h"            // effects, helpers
#include "AmigaRulez.h"     // boing ball animation (projected)
#include "qoi_skull_anim.h" // spinning heart animation (qoi zoetrope)
#include "qoi_heart_anim.h" // spinning skull animation (qoi zoetrope)


enum AnimationTypes
{
  ANIMATION_AMIGA,
  ANIMATION_QOI
};

// set this to display the heart animation or the amiga ball
AnimationTypes AnimationType;// = ANIMATION_AMIGA;
Animation* animation;
AmigaRulez* AmigaBall = new AmigaRulez;
Button *Btns[3] = { &M5.BtnA, &M5.BtnB, &M5.BtnC };
RGBColor linesColor = { 255, 192, 128 };
static uint8_t chroma = 0, lastchroma = 0;
static uint8_t *rgb = (uint8_t*)&linesColor;


int aframe = 0; // animation frame number, increased by time unless capturing

uint16_t spritePosX, spritePosY;
uint16_t corePosX, corePosY;
uint16_t centerX, centerY;

uint16_t AmigaBallWidth  = 48;
uint16_t AmigaBallHeight = 48;

uint32_t lastpress = millis();

float renderangle = 0, renderzoomx = 1.0, renderzoomy = 1.0;

typedef void(*btnAction_t)(void);



static void calcLinesFromPoint( uint8_t x, uint8_t y, uint8_t z, uint8_t w )
{

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
    sprite.setCursor( 0, 0 );
    sprite.printf("X:%02x Y:%02x Z:%02x W:%02x", x, y, z, w );
    sprite.pushSprite( spritePosX, spritePosY );
    linePush( {pointIndex, line1Index} );
    linePush( {pointIndex, line2Index} );
    linePush( {pointIndex, line3Index} );
    linePush( {pointIndex, line4Index} );
    pointPush( pointIndex );
  }
  transformPoint({x, y, z, w});
}


static void drawPoints()
{

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

        colorstart.set( byte(linesColor.r-valstart/2), byte(linesColor.g-valstart/2), byte(linesColor.b-valstart/2) );
        colorend.set  ( byte(linesColor.r-valend/2),   byte(linesColor.g-valend/2),   byte(linesColor.b-valend/2) );

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

    if( i==7 ) { // z-depth absmiddle
      sprite.pushImage( corePosX, corePosY, coreSprite.width(), coreSprite.height(), (uint16_t*)coreSprite.getBuffer(), TFT_BLACK );
    }

    float r = sphereMass + cache4Dbuff[srtindex][2]*scaletobyte*depthScale;
    float c = map( cache4Dbuff[srtindex][2], 0, 255, 192, 255 );
    uint16_t sphereColor = tft.color565( c,c,c );
    Coords3D sphereCoords = { cache4Dbuff[srtindex][0], cache4Dbuff[srtindex][1], cache4Dbuff[srtindex][2]};
    drawCache3dSphere( &sprite, sphereCoords, r*2., sphereColor );
  }
}



//uint8_t* rgbBuffer = NULL;
LGFX_Sprite spinSprite( &tft );
uint16_t* blahPtr = NULL;

static void drawTesseract()
{
  unsigned long time_before_draw = millis();

  if( dostrobe ) {
    spriteFadeOut( &sprite, 192 );
  } else {
    sprite.fillSprite( TFT_BLACK );
  }

  drawPoints();

  if( drawfps ) {
    sprite.setCursor( 0, 0 );
    sprite.printf("fps: %2.2f\n", fps);
    sprite.printf("balls: %d", sphereCache.size()+1);
    //sprite.printf("%02x %02x %02x ", linesColor.r, linesColor.g, linesColor.b );
  }

  //sprite.pushSprite( spritePosX, spritePosY );
  sprite.pushRotateZoom/*WithAA*/( &tft, centerX, centerY, renderangle, renderzoomx, renderzoomy );

  unsigned long time_before_capture = millis();

  #ifdef CAPTURE_MODE
    uint32_t now = millis();
    char fileName[32];
    sprintf( fileName, "/qoi/tesseract-%03d.qoi", captured );
    fs::File file = SD.open( fileName, FILE_WRITE, true );
    if ( file ) {
      qoi_desc desc;
      qoi_encode( &file, &sprite, &desc );
      size_t size = file.size();
      file.close();
      log_d("Screenshot saved as %s (%d bytes). Total time %d ms", fileName, size, millis()-now );
    }
    //sprintf( fileName, "/bmp/tesseract-%03d.bmp", captured );
    //M5.ScreenShot->snapBMP( fileName );
    vTaskDelay(1);
  #endif

  unsigned long time_to_capture = millis() - time_before_capture;

  captured++;
  fpscount++;

  aframe++;
  //aframe = map( referenceAngle*1000, 0, TwoPi*1000, 0, (framesCount*2)-1);
  aframe = aframe%framesCount;
  int currentframe = framesCount-(aframe+1);
  //Serial.println( currentframe );

  switch( AnimationType ) {
    case ANIMATION_AMIGA:
      // spin axis 1
      AmigaBall->Phase = fmod( AmigaBall->Phase + ( AmigaBall->phase4Rad - AmigaBall->PhaseVelocity ), AmigaBall->phase4Rad );
      // spin axis 2
      AmigaBall->TiltRad += .01;
      AmigaBall->drawBall( AmigaBall->Phase, coreSprite.width()/2, coreSprite.height()/2, coreSprite.width()/2, coreSprite.height()/2);
    break;
    case ANIMATION_QOI:
      switch( animation->type ) {
        case IMG_JPG: coreSprite.drawJpg( animation->frame(currentframe).data, animation->frame(currentframe).data_len, 0, 0 ); break;
        case IMG_PNG: coreSprite.drawPng( animation->frame(currentframe).data, animation->frame(currentframe).data_len, 0, 0 ); break;
        case IMG_QOI: coreSprite.drawQoi( animation->frame(currentframe).data, animation->frame(currentframe).data_len, 0, 0 ); break;
        default:
          break;
      }
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


static void doCalcCoords()
{
  referenceAngle = timetraveller/*millis()*/ / /*2500.0*/ 1250.0 /*625.0*/;
  referenceAngle = fmod( referenceAngle, FourPi );

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


static void calcCoordsTask( void * param )
{
  while( 1 ) {
    // prevent a draw to begin as data will
    doCalcCoords();
    vTaskDelay( 1 );
  }
  vTaskDelete( NULL );
}


static void setAnimation( Animation* anim )
{
  animation = anim;
  coreSprite.deleteSprite();
  if(! coreSprite.createSprite( animation->width(), animation->height() ) ) {
    log_e("Failed to create %dx%d animation sprite, halting", animation->width(), animation->height() );
    while(1) vTaskDelay(1);
  }
  framesCount = animation->framesCount;
}


static void setAmigaBall()
{
  coreSprite.deleteSprite();
  if(! coreSprite.createSprite( AmigaBallWidth, AmigaBallHeight ) ) {
    log_e("Failed to create %dx%d amigaball sprite, halting", AmigaBallWidth, AmigaBallHeight );
    while(1) vTaskDelay(1);
  }
}


static btnAction_t btnActions[3] =
{
  []() { // BtnA: toggle fps counter
    drawfps = !drawfps;
    lastpress = millis();
    log_d("Draw fps: %s", drawfps?"true":"false");
  },

  []() { // BtnB: toggle strobe effect
    dostrobe = !dostrobe;
    lastpress = millis();
    log_d("Strobe: %s", dostrobe?"true":"false");
  },

  []() { // BtnC: toggle amigaball / heart animation
    AnimationType = (AnimationType==ANIMATION_AMIGA)?ANIMATION_QOI:ANIMATION_AMIGA;
    if( AnimationType==ANIMATION_QOI ) {
      setAnimation( animation==std::addressof(animation1) ? &animation2 : &animation1 );
    } else {
      setAmigaBall();
    }
    coreSprite.fillSprite( TFT_BLACK );
    corePosX = sprite.width()/2-(coreSprite.width()/2);
    corePosY = sprite.height()/2-(coreSprite.height()/2);
    lastpress = millis();
  }

};






static void mainTask( void * param )
{

  clearCoordsCache();

  xTaskCreatePinnedToCore( calcCoordsTask, "calcCoordsTask", 2048, NULL, 0, NULL, 1 ); /* last = Task Core */
  while( !needsdrawing ) { vTaskDelay(1); } // wait for first frame

  while(1)
  {
    if( needsdrawing ) {
      needsdrawing = false;
      drawing = true;

      chroma = (lastchroma+1)%3;

      if( rgb[chroma] < 255 && rgb[lastchroma] > 128 ) {
        rgb[lastchroma]--;
        rgb[chroma]++;
      } else {
        lastchroma = chroma;
      }

      //renderangle += .5;
      //renderzoomx
      //renderzoomy

      drawTesseract();
      drawing = false;
    }

    M5.update();

    bool waspressed = false;
    for( int i=0;i<3;i++ ) {
      if( Btns[i]->wasPressed() ) {
        btnActions[i]();
        waspressed = true;
      }
    }

    if( ! waspressed ) {

      if( millis() - lastpress > 3000 ) {
        uint8_t btnId = map( random()%1024, 0, 1024, 1, 2 );
        btnActions[btnId]();
        lastpress = millis();
      }

    }

    vTaskDelay(1);
  }
  vTaskDelete( NULL );
}




void setup()
{
  M5.begin( true, true, true, false, ScreenShotEnable );

  checkSDUpdater( SD, MENU_BIN, 5000, TFCARD_CS_PIN );

  sprite.setPsram( false );
  sprite.setColorDepth( 16 ); // set this to 8 when screenWidth*screenHeight > 230*230, since pram is disabled for sprites in this demo

  coreSprite.setPsram( false );
  coreSprite.setColorDepth( 16 );

  if(! sprite.createSprite( screenWidth, screenHeight ) ) {
    log_e("Failed to create %dx%d main sprite, halting", screenWidth, screenHeight );
    while(1) vTaskDelay(1);
  }

  spritePosX = tft.width()/2 - screenWidth/2;
  spritePosY = tft.height()/2 - screenHeight/2;
  sprite.fillSprite( TFT_BLACK );

  amigaBallConfig.Width      = AmigaBallWidth;
  amigaBallConfig.Height     = AmigaBallHeight;
  amigaBallConfig.ScaleRatio = 5;
  amigaBallConfig.sprite     = &coreSprite;
  AmigaBall->init( amigaBallConfig );

  setAmigaBall();

  corePosX = sprite.width()/2-(coreSprite.width()/2);
  corePosY = sprite.height()/2-(coreSprite.height()/2);

  centerX = tft.width()/2;
  centerY = tft.height()/2;

  // sprite.pushSprite( 0, 0 );

  aframe++;
  timetraveller = 0;

  dostrobe = false;
  drawfps = true;
  AnimationType = ANIMATION_AMIGA;

  #ifndef CAPTURE_MODE
    xTaskCreatePinnedToCore( mainTask, "mainTask", 4096, NULL, 32, NULL, 0 ); /* last = Task Core */
  #endif

}


void loop()
{
  #ifdef CAPTURE_MODE
    dostrobe = true;
    drawfps = true;
    // capturing display from task doesn't work well
    // so this is done from the main loop instead
    doCalcCoords();

    if( rgb[chroma] < 255 && rgb[lastchroma] > 128 ) {
      rgb[lastchroma]--;
      rgb[chroma]++;
    } else {
      lastchroma = chroma;
    }

    drawTesseract();
  #else
    vTaskSuspend(NULL);
  #endif
}
