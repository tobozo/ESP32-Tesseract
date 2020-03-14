/*

  ESP32 AmigaBoingBall - A port of the famous Amiga Boing Ball Demo
  ported from https://github.com/niklasekstrom/boing_ball_python/
  Source: https://github.com/tobozo/ESP32-AmigaBoingBall

  MIT License

  Copyright (c) 2019 tobozo

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

*/


struct AmigaBallConfig {
  long Framelength = 20;
  byte Wires = 7; // 0 = no wireframes
  uint16_t BGColor = tft.color565(0xa9, 0xa9, 0xa9);
  uint16_t GridColor =  tft.color565(0xac, 0x00, 0xac);
  uint16_t ShadowColor = tft.color565(0x66, 0x66, 0x66);
  uint16_t YPos = 0;
  uint16_t XPos = 0;
  uint16_t Width = tft.width();
  uint16_t Height = tft.height();
  uint16_t ScaleRatio = 5; // ball size will have this/nth of the window Height, bigger value means smaller ball
  TFT_eSprite *sprite;
} amigaBallConfig;


struct Points {
  float x = 0.00;
  float y = 0.00;
};

struct AmigaRulez {

  TFT_eSprite *sprite;

  Points points[10][10];

  float deg2rad   = PI/180.0;
  float phase8Rad = PI/8.0; // 22.5 deg
  float phase4Rad = PI/4.0; // 45 deg
  float phase2Rad = PI/2.0; // 90 deg
  float twopi     = PI*2;
  float Phase     = 0.0;
  float velocityX = 2.1;
  float velocityY = 0.07;
  float angleY    = 0.0;

  float PhaseVelocity;
  float perspective[4];
  float XtoYratio;
  float YtoXratio;
  float TiltRad;

  bool AnimationDone;
  bool isMovingRight;
  bool isMovingUp = false;

  byte Wires;
  byte bytecounter = 0;

  int BounceMargin;

  long Framelength;
  long startedTick = millis();
  long lastTick    = millis();
  long processTicks = 0;

  float variableScale = Scale;
  float oldScale = Scale;
  float ScaleAmplitude = 8;
  float MaxScaleAmplitude;
  float AmplitudeFactor = 4;
  float TiltDeg = 72; // 17 degrees tilting to the right
  float LeftBoundary;
  float RightBoundary;
  float ShadowYPos = 0.00;
  float XPos;
  float YPos;
  float Width;
  float Height;

  float VCentering;
  float Scale;
  float YPosAmplitude;
  uint16_t ScaleRatio;
  uint16_t BGColor;
  uint16_t GridColor;
  uint16_t ShadowColor;
  float lastPositionX;
  float lastPositionY;
  float positionX;
  float positionY;

  int spriteWidth;
  int spriteHeight;
  int spriteCenterX;
  int spriteCenterY;

  void init( AmigaBallConfig config = amigaBallConfig ) {
    BGColor     = config.BGColor;
    GridColor   = config.GridColor;
    Framelength = config.Framelength;//33; // millis
    ScaleRatio  = config.ScaleRatio;
    ShadowColor = config.ShadowColor;
    XPos   = config.XPos;
    YPos   = config.YPos;
    Width  = config.Width;
    Height = config.Height;
    Wires  = config.Wires;
    sprite = config.sprite;
    setupValues();
  }


  void setupValues() {
    Scale = Height/ScaleRatio;//
    ScaleAmplitude = Scale/ AmplitudeFactor; // ball diameter will vary on this
    MaxScaleAmplitude = Scale + ScaleAmplitude;

    spriteWidth  = (MaxScaleAmplitude + AmplitudeFactor )*2;
    spriteHeight = (MaxScaleAmplitude + AmplitudeFactor )*2;

    spriteCenterX = spriteWidth/2 + spriteWidth%2;
    spriteCenterY = spriteHeight/2 - spriteHeight%2;

    YPosAmplitude = (Height-(Scale+ScaleAmplitude))/2; // ball will bounce on this span pixels
    VCentering = YPos + (Height-1) - (MaxScaleAmplitude + AmplitudeFactor);// -(YPosAmplitude/2 + Scale + ScaleAmplitude);

    BounceMargin = AmplitudeFactor*2+Scale+ScaleAmplitude; // 135
    LeftBoundary = XPos + BounceMargin;
    RightBoundary = XPos + Width - BounceMargin;

    TiltRad = TiltDeg * deg2rad;
    lastPositionX = 0;
    lastPositionY = 0;
    PhaseVelocity = 2.5 * deg2rad;
    positionX = XPos + Width/2;
    isMovingRight = true;
  }

  float getLat(float phase, int i) {
    if(i == 0) {
      return -phase2Rad;
    } else if(i == 9) {
      return phase2Rad;
    } else {
      return -phase2Rad + phase + (i-1) * phase8Rad;
    }
  }

  void calcPoints(float phase) {
    float sin_lat[10] = {0};// = {}
    for(int i=0;i<10;i++) {
      float lat = getLat(phase, i);
      sin_lat[i] = sin( lat );
    }
    for(int j=0;j<9;j++) {
      float lon = -phase2Rad + j * phase8Rad;
      float _y = sin( lon );
      float _l = cos( lon );
      for(int i=0;i<10;i++) {
        float _x = sin_lat[i] * _l;
        points[i][j].x = _x;
        points[i][j].y = _y;
      }
    }
  }

  void tiltSphere(float ang) {
    float st = sin( ang );
    float ct = cos( ang );
    for( int i=0; i<10; i++) {
      for( int j=0; j<9; j++) {
        float _x = points[i][j].x * ct - points[i][j].y * st;
        float _y = points[i][j].x * st + points[i][j].y * ct;
        points[i][j].x = _x;
        points[i][j].y = _y;
      }
    }
  }

  void scaleTranslate(float s, float tx, float ty) {
    for( int i=0; i<10; i++) {
      for( int j=0; j<9; j++ ) {
        float _x = points[i][j].x * s + tx;
        float _y = points[i][j].y * s + ty;
        points[i][j].x = _x;
        points[i][j].y = _y;
      }
    }
  }

  void transform(float s, float tx, float ty) {
    tiltSphere( TiltRad );
    scaleTranslate( s, tx, ty );
  }

  void fillTiles(bool alter) {
    for( int j=0; j<8; j++ ) {
      for( int i=0; i<9; i++) {
        uint16_t color = alter ? TFT_RED : TFT_WHITE;
        sprite->fillTriangle(points[i][j].x,     points[i][j].y,     points[i+1][j].x, points[i+1][j].y, points[i+1][j+1].x, points[i+1][j+1].y, color);
        sprite->fillTriangle(points[i+1][j+1].x, points[i+1][j+1].y, points[i][j+1].x, points[i][j+1].y, points[i][j].x,     points[i][j].y,     color);
        alter = !alter;
      }
    }
  }

  void drawBall(float phase, float scale, float oldscale, float x, float y) {
    calcPoints( fmod(phase, phase8Rad) );
    transform(scale, x, y);
    fillTiles(phase >= phase8Rad);
  }

} AmigaBall;
