/*\
 *

  ESP32-Tesseract gfx functions

  A complement to any TFT/sprite demo brought to you by tobozo

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-label"
#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wempty-body"
#pragma GCC diagnostic ignored "-Wunused-function"

#define NUM_LEVELS 256
float Draw_gamma = 2.35;   /* looks good enough */
bool gamma_set = false;
unsigned char gamma_table[NUM_LEVELS];
int32_t spriteCenterX;
int32_t spriteCenterY;

TFT_eSprite tmpsprite = TFT_eSprite(&tft);
TFT_eSprite sprite = TFT_eSprite( &tft );
TFT_eSprite coreSprite = TFT_eSprite( &tft );


typedef std::vector<std::vector<float>> PointsArray;
typedef std::array<float,4> Coords;
typedef std::array<uint8_t,4> ByteCoords;

typedef std::vector<std::array<int16_t,2>> LinesIndexesArray;
typedef std::vector<int16_t> PointsIndexesArray;
typedef std::array<int16_t,4> Coords4D;
typedef std::array<int16_t,3> Coords3D;


void initGamma( int nlevels ) {
  for (i=0; i<nlevels; i++) {
    gamma_table[i] = (int) (nlevels-1)*pow((float)i/((float)(nlevels-1)), 1.0/Draw_gamma);
  }
  gamma_set = true;
}


RGBColor colorAt( int32_t start, int32_t end, int32_t pos, RGBColor colorstart, RGBColor colorend ) {
  if( pos == end ) return colorend;
  if( pos == start ) return colorstart;
  uint8_t r,g,b;
  r = map( pos, start, end, colorstart.r, colorend.r );
  g = map( pos, start, end, colorstart.g, colorend.g );
  b = map( pos, start, end, colorstart.b, colorend.b );
  return {r,g,b};
}


RGBColor bgColorAt( TFT_eSprite *sprite, int32_t x, int32_t y ) {
  if( x < 0 || y < 0 || x >= sprite->width() || y >= sprite->height() ) {
    return {0,0,0};
  }
  RGBColor bgcolor;
  uint16_t color565 = sprite->readPixel( x, y );
  color565 = (color565 << 8) | (color565 >> 8);
  bgcolor.set( color565 );
  return bgcolor;
}


uint16_t gammaColorAt( TFT_eSprite *sprite, int32_t x, int32_t y, RGBColor pixelcolor, uint16_t wgt ) {
  if( x < 0 || y < 0 || x >= sprite->width() || y >= sprite->height() ) {
    return tft.color565( pixelcolor.r, pixelcolor.g, pixelcolor.b);
  }
  RGBColor bgcolor = bgColorAt( sprite, x, y );
  pixelcolor.r = pixelcolor.r - (wgt*(pixelcolor.r-bgcolor.r)/255);
  pixelcolor.g = pixelcolor.g - (wgt*(pixelcolor.g-bgcolor.g)/255);
  pixelcolor.b = pixelcolor.b - (wgt*(pixelcolor.b-bgcolor.b)/255);
  return tft.color565( pixelcolor.r, pixelcolor.g, pixelcolor.b);
}


// draw antialiased line, inspired from Xiaolin Wu algorithm
static void drawLineAntialiased( TFT_eSprite *sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, RGBColor fgcolor, int nlevels=NUM_LEVELS, int nbits=8) {
  uint16_t intshift, erracc,erradj;
  uint16_t erracctmp, wgt, wgtcompmask, wgtpow;
  uint16_t fg16 = tft.color565(fgcolor.r, fgcolor.g, fgcolor.b );
  int dx, dy, tmp, xdir;
  if( !gamma_set ) {
    initGamma( NUM_LEVELS );
  }
  if (y0 > y1) {
    tmp = y0; y0 = y1; y1 = tmp;
    tmp = x0; x0 = x1; x1 = tmp;
  }
  // draw the initial pixel in the foreground color
  sprite->drawPixel(x0, y0, fg16 );
  dx = x1 - x0;
  xdir = (dx >= 0) ? 1 : -1;
  dx = (dx >= 0) ? dx : -dx;
  // special-case horizontal, vertical, and diagonal lines which need no
  // weighting because they go right through the center of every pixel.
  if ((dy = y1 - y0) == 0) {
    sprite->drawFastHLine( (x0 < x1) ? x0 : x1, y0, dx, fg16 );
    return;
  }
  if (dx == 0) {
    sprite->drawFastVLine( x0, y0, dy, fg16 );
    return;
  }
  if (dx == dy) {
    for (; dy != 0; dy--) {
      x0 += xdir;
      y0++;
      sprite->drawPixel(x0, y0, fg16 );
    }
    return;
  }
  // line is not horizontal, vertical, or diagonal
  erracc = 0; // err. acc. is initially zero
  // # of bits by which to shift erracc to get intensity level
  intshift = 16 - nbits;
  // mask used to flip all bits in an intensity weighting
  wgtcompmask = nlevels - 1;
  // x-major or y-major?
  if (dy > dx) {
    // y-major.  Calculate 16-bit fixed point fractional part of a pixel that
    // X advances every time Y advances 1 pixel, truncating the result so that
    // we won't overrun the endpoint along the X axis
    erradj = ((uint32_t)dx << 16) / (uint32_t)dy;
    // draw all pixels other than the first and last
    while (--dy) {
      erracctmp = erracc;
      erracc += erradj;
      if (erracc <= erracctmp) {
        // rollover in error accumulator, x coord advances
        x0 += xdir;
      }
      // y-major so always advance Y
      y0++;
      // the nbits most significant bits of erracc give us the intensit
      // weighting for this pixel, and the complement of the weighting for
      // the paired pixel.
      wgt = erracc >> intshift;
      // generate the colors by linear interpolation, applying gamma correction
      sprite->drawPixel( x0, y0, gammaColorAt( sprite, x0, y0, fgcolor, wgt ) );

      wgtpow = wgt^wgtcompmask;
      // generate the colors by linear interpolation, applying gamma correction
      sprite->drawPixel( x0+xdir, y0, gammaColorAt( sprite, x0+xdir, y0, fgcolor, wgtpow ) );
    }
    // draw the final pixel, which is always exactly intersected by the line
    // and so needs no weighting */
    sprite->drawPixel(x1, y1, fg16 );
    return;
  }
  // x-major line.  Calculate 16-bit fixed-point fractional part of a pixel
  // that Y advances each time X advances 1 pixel, truncating the result so
  // that we won't overrun the endpoint along the X axis.
  erradj = ((uint32_t)dy << 16) / (uint32_t)dx;
  // draw all pixels other than the first and last
  while (--dx) {
    erracctmp = erracc;
    erracc += erradj;
    if (erracc <= erracctmp) {
      // accumulator turned over, advance y
      y0++;
    }
    x0 += xdir; // x-major so always advance X
    // the nbits most significant bits of erracc give us the intensity
    // weighting for this pixel, and the complement of the weighting for
    // the paired pixel.
    wgt = erracc >> intshift;
    // generate the colors by linear interpolation, applying gamma correction
    sprite->drawPixel( x0, y0, gammaColorAt( sprite, x0, y0, fgcolor, wgt) );

    wgtpow = wgt^wgtcompmask;
    // generate the colors by linear interpolation, applying gamma correction
    sprite->drawPixel( x0, y0+1, gammaColorAt( sprite, x0, y0+1, fgcolor, wgtpow ) );
  }
  // draw final pixel, always exactly intersected by the line and doesn't
  // need to be weighted.
  sprite->drawPixel(x1, y1, fg16 );
}


// draw antialiased *gradient* line, inspired from Xiaolin Wu algorithm (added gradient support)
static void drawGradientLineAntialiased( TFT_eSprite *sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, RGBColor colorstart, RGBColor colorend, int nlevels=NUM_LEVELS, int nbits=8) {
  uint16_t intshift, erracc,erradj;
  uint16_t erracctmp, wgt, wgtcompmask, wgtpow;
  int dx, dy, tmp, xdir;
  const int32_t _x0 = x0, _x1 = x1, _y0 = y0, _y1 = y1; // freeze values
  RGBColor pixelcolor;

  if( !gamma_set ) {
    initGamma( NUM_LEVELS );
  }
  if (y0 > y1) {
    tmp = y0; y0 = y1; y1 = tmp;
    tmp = x0; x0 = x1; x1 = tmp;
    swap_coord(colorstart, colorend);
  }
  // draw the initial pixel in the foreground color
  sprite->drawPixel(x0, y0, tft.color565(colorstart.r, colorstart.g, colorstart.b ) );
  dx = x1 - x0;
  xdir = (dx >= 0) ? 1 : -1;
  dx = (dx >= 0) ? dx : -dx;
  // special-case horizontal, vertical, and diagonal lines which need no
  // weighting because they go right through the center of every pixel.
  if ((dy = y1 - y0) == 0) {
    //sprite.drawFastHLine( (x0 < x1) ? x0 : x1, y0, dx, fg16 );
    sprite->drawGradientHLine( (x0 < x1) ? x0 : x1, y0, dx, colorstart, colorend );
    return;
  }
  if (dx == 0) {
    sprite->drawGradientVLine( x0, y0, dy, colorstart, colorend );
    return;
  }
  if (dx == dy) {
    for (; dy != 0; dy--) {
      x0 += xdir;
      y0++;
      pixelcolor = colorAt( _x0, _x1, x0, colorstart, colorend );
      sprite->drawPixel(x0, y0, tft.color565(pixelcolor.r, pixelcolor.g, pixelcolor.b ) );
    }
    return;
  }
  pixelcolor = colorstart;
  // line is not horizontal, vertical, or diagonal
  erracc = 0; // err. acc. is initially zero
  // # of bits by which to shift erracc to get intensity level
  intshift = 16 - nbits;
  // mask used to flip all bits in an intensity weighting
  wgtcompmask = nlevels - 1;
  // x-major or y-major?
  if (dy > dx) {
    // y-major.  Calculate 16-bit fixed point fractional part of a pixel that
    // X advances every time Y advances 1 pixel, truncating the result so that
    // we won't overrun the endpoint along the X axis
    erradj = ((uint32_t)dx << 16) / (uint32_t)dy;
    // draw all pixels other than the first and last
    while (--dy) {
      erracctmp = erracc;
      erracc += erradj;
      if (erracc <= erracctmp) {
        // rollover in error accumulator, x coord advances
        x0 += xdir;
      }
      // y-major so always advance Y
      y0++;
      pixelcolor = colorAt( _y0, _y1, y0, colorstart, colorend );
      // the nbits most significant bits of erracc give us the intensit
      // weighting for this pixel, and the complement of the weighting for
      // the paired pixel.
      wgt = erracc >> intshift;
      // generate the colors by linear interpolation, applying gamma correction
      sprite->drawPixel( x0, y0, gammaColorAt( sprite, x0, y0, pixelcolor, wgt ) );

      wgtpow = wgt^wgtcompmask;
      // generate the colors by linear interpolation, applying gamma correction
      sprite->drawPixel( x0+xdir, y0, gammaColorAt( sprite, x0+xdir, y0, pixelcolor, wgtpow ) );
    }
    // draw the final pixel, which is always exactly intersected by the line
    // and so needs no weighting */
    sprite->drawPixel(x1, y1, tft.color565( colorend.r, colorend.g, colorend.b ) );
    return;
  }
  // x-major line.  Calculate 16-bit fixed-point fractional part of a pixel
  // that Y advances each time X advances 1 pixel, truncating the result so
  // that we won't overrun the endpoint along the X axis.
  erradj = ((uint32_t)dy << 16) / (uint32_t)dx;
  // draw all pixels other than the first and last
  while (--dx) {
    erracctmp = erracc;
    erracc += erradj;
    if (erracc <= erracctmp) {
      // accumulator turned over, advance y
      y0++;
    }
    x0 += xdir; // x-major so always advance X
    pixelcolor = colorAt( _x0, _x1, x0, colorstart, colorend );
    // the nbits most significant bits of erracc give us the intensity
    // weighting for this pixel, and the complement of the weighting for
    // the paired pixel.
    wgt = erracc >> intshift;
    // generate the colors by linear interpolation, applying gamma correction
    sprite->drawPixel( x0, y0, gammaColorAt( sprite, x0, y0, pixelcolor, wgt ) );

    wgtpow = wgt^wgtcompmask;
    // generate the colors by linear interpolation, applying gamma correction
    sprite->drawPixel( x0, y0+1, gammaColorAt( sprite, x0, y0+1, pixelcolor, wgtpow ) );
  }
  // draw final pixel, always exactly intersected by the line and doesn't
  // need to be weighted.
  sprite->drawPixel(x1, y1, tft.color565( colorend.r, colorend.g, colorend.b ) );
}


// Calculates distance for radius0 and radius1 from any given line, then draws partial *gradient* line:
//
// [-------------------------------------- line --------------------------------------]
//
// [-- radius0 --][------------------- partial line -------------------][-- radius1 --]
//
static void drawPartialGradientLineAntialiased( TFT_eSprite *sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, RGBColor colorstart, RGBColor colorend, int16_t radius0, int16_t radius1 ) {
  float d  /* distance */        = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) );
  float t0 /* distanceratio 0 */ = 1.3*radius0 / d;
  float t1 /* distanceratio 1 */ = 1.3*radius1 / d;

  int32_t x2, y2, x3, y3;

  x2 = ((1.0-t0)*float(x0) + t0*float(x1));
  y2 = ((1.0-t0)*float(y0) + t0*float(y1));

  x3 = ((1.0-t1)*x1 + t1*x0);
  y3 = ((1.0-t1)*y1 + t1*y0);

  drawGradientLineAntialiased( sprite, x2, y2, x3, y3, colorstart, colorend );
}


// Calculates distance for radius0 and radius1 from any given line, then draws partial *non gradient* line:
//
// [-------------------------------------- line --------------------------------------]
//
// [-- radius0 --][------------------- partial line -------------------][-- radius1 --]
//
__attribute__((unused))
static void drawPartialLineAntialiased( TFT_eSprite *sprite, int32_t x0, int32_t y0, int32_t x1, int32_t y1, RGBColor pixelcolor, int16_t radius0, int16_t radius1 ) {
  float d  /* distance */        = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) );
  float t0 /* distanceratio 0 */ = 1.3*radius0 / d;
  float t1 /* distanceratio 1 */ = 1.3*radius1 / d;

  int32_t x2, y2, x3, y3;

  x2 = ((1.0-t0)*float(x0) + t0*float(x1));
  y2 = ((1.0-t0)*float(y0) + t0*float(y1));

  x3 = ((1.0-t1)*x1 + t1*x0);
  y3 = ((1.0-t1)*y1 + t1*y0);

  drawLineAntialiased( sprite, x2, y2, x3, y3, pixelcolor );
}



// luminosity reducer, taken from Bodmer's antialiased font
static uint16_t luminance(uint16_t color, uint8_t luminance) {
  // Extract rgb colours and stretch range to 0 - 255
  uint16_t r = (color & 0xF800) >> 8; r |= (r >> 5);
  uint16_t g = (color & 0x07E0) >> 3; g |= (g >> 6);
  uint16_t b = (color & 0x001F) << 3; b |= (b >> 5);

  b = ((b * (uint16_t)luminance + 255) >> 8) & 0x00F8;
  g = ((g * (uint16_t)luminance + 255) >> 8) & 0x00FC;
  r = ((r * (uint16_t)luminance + 255) >> 8) & 0x00F8;

  return (r << 8) | (g << 3) | (b >> 3);
}


// Use as a gfx replacement for tft.fillScreen(TFT_BLACK).
// Produces a strobe effect by reducing luminosity on all non-black pixels.
// Limitations:
//  - 16bits colors only
//  - Only fades to black
//  - frame rate must be >15fps
//  - Sluggish when the sprite has too many (>50%) non black pixels
static void spriteFadeOut( TFT_eSprite *sprite, uint8_t strength ) {
  int32_t w = sprite->width();
  int32_t h = sprite->height();
  int32_t l = w*h;
  uint16_t* framebuf = (uint16_t*)sprite->frameBuffer(0);
  for( uint32_t i=0;i<l;i++) {
    if( framebuf[i]!=TFT_BLACK ) {
      uint16_t pixcolor = (framebuf[i] >> 8) | (framebuf[i] << 8);
      pixcolor = luminance( pixcolor, strength );
      framebuf[i] = pixcolor<<8 | pixcolor>>8;
    }
  }
}


// ******************************
//       Raytraced Balls !
//            ____
//        ,dP9CGG88@b,
//      ,IP""YICCG888@@b,
//      dIi   ,IICGG8888@b
//    dCIIiciIICCGG8888@@b
//    GCCIIIICCCGGG8888@@@.
//    GGCCCCCCCGGG88888@@@.
//    GGGGCCCGGGG88888@@@@.
//    Y8GGGGGG8888888@@@@P
//      Y88888888888@@@@@P
//      `Y8888888@@@@@@@P'
//        `@@@@@@@@@P'
//            """"
//
// ******************************

// Raytracing is a bit too slow for realtime, so
// once a ball has been rendered, it is cached
// as an 8bit intensity pixel map.


// fixed light source, applied relativaly to all balls
// this needs to be normalized once at runtime
float fixedlight[3] = { 0, 75, 0 };
static bool normalized = false;

// spheres are indexed by radius and light source
typedef struct indexCache {
  int32_t radius;
  int8_t light[3];
} IndexCache;

// actually more intensity than color
typedef std::vector<uint8_t> PixelColors;

struct SphereCache {
  int16_t width;
  int16_t height;
  uint16_t radius;
  int16_t offset;
  PixelColors pixels;
  int8_t light[3];
};

typedef std::vector<SphereCache> SphereCacheArray;

SphereCacheArray sphereCache;

enum Symetry {
  SYM_NONE,
  SYM_X
};

struct SphereRef {
  int index;
  Symetry symetry;
};


// finds a sphere (or its symetrical version) in the cache
// returns { index in cache, symetry direction }
SphereRef findCachedSphere( const SphereCacheArray &haystack, const IndexCache &needle ) {
  int max=haystack.size();
  if (max==0) return { -1, SYM_NONE };
  for(int i=0; i<max; i++)
    if ( haystack[i].radius   == needle.radius
      && haystack[i].light[0] == needle.light[0]
      && haystack[i].light[1] == needle.light[1]
      && haystack[i].light[2] == needle.light[2] )
      return { i, SYM_NONE };
    else if ( haystack[i].radius   ==  needle.radius
      && haystack[i].light[0] == -needle.light[0]
      && haystack[i].light[1] ==  needle.light[1]
      && haystack[i].light[2] ==  needle.light[2] )
      return { i, SYM_X };
  return { -1, SYM_NONE };
}


void normalize(float * v) {
  float len = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  v[0] /= len; v[1] /= len; v[2] /= len;
}

float dot(float *x, float *y) {
  float d = x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
  return d < 0 ? -d : 0;
}


// render a cached sphere
void renderCachedSphere( TFT_eSprite *sprite, SphereRef sphereRef, int16_t cornerx, int16_t cornery, uint16_t color ) {
  int sphereIndex = sphereRef.index;
  tmpsprite.createSprite( sphereCache[sphereIndex].width, sphereCache[sphereIndex].height );
  tmpsprite.setColorDepth( 16 );
  tmpsprite.setWindow( 0, 0, tmpsprite.width(), tmpsprite.height() );

  int32_t loopsize = sphereCache[sphereIndex].pixels.size();

  switch( sphereRef.symetry ) {
    case SYM_NONE:
      for( uint32_t i=0; i < loopsize; i++ ) {
        uint16_t _color = luminance( color, sphereCache[sphereIndex].pixels[i] );
        tmpsprite.pushColor( _color );
      }
    break;
    case SYM_X:
      for( uint16_t yy = 0; yy < sphereCache[sphereIndex].height; yy++ ) {
        for( int16_t xx = sphereCache[sphereIndex].width - 1; xx >= 0; xx-- ) {
          uint16_t pixelpos = xx + yy*sphereCache[sphereIndex].width;
          uint16_t _color = luminance( color, sphereCache[sphereIndex].pixels[pixelpos] );
          tmpsprite.pushColor( _color );
        }
      }
    break;
  }

  sprite->pushImage(
    sphereCache[sphereIndex].offset + cornerx - 2,
    sphereCache[sphereIndex].offset + cornery - 2,
    sphereCache[sphereIndex].width,
    sphereCache[sphereIndex].height,
    (uint16_t*)tmpsprite.frameBuffer(0),
    TFT_BLACK
  );
  tmpsprite.deleteSprite();
}


// cache and/or draw
void drawCache3dSphere( TFT_eSprite *sprite, Coords3D coords3d, uint16_t diameter, uint16_t color=TFT_WHITE, float k=4, float ambient=0.5) {
  uint16_t radius = diameter *.5;
  int i, j, intensity, sphereIndex;
  float bfixed, bvar;
  float vec[3], x, y;
  float radiuspow = radius * radius;
  float radius10 = radius*10.;
  int32_t posx = coords3d[0];
  int32_t posy = coords3d[1];
  int32_t posz = coords3d[2];

  if( ! normalized ) {
    // fixed 'relative' light source only needs to be normalized once
    normalized = true;
    normalize(fixedlight);
  }

  spriteCenterX = sprite->width()  *.5;
  spriteCenterY = sprite->height() *.5;

  // reduce light source to short range indexes for caching
  int8_t indexedlight[3] = {
    (int8_t)map( spriteCenterX - posx, -spriteCenterX, spriteCenterX, 16, -16 ),
    (int8_t)map( spriteCenterY - posy, -spriteCenterX, spriteCenterX, 16, -16 ),
    (int8_t)map( posz, 0, 255, -16, 16 )
  };

  tmpsprite.setAttribute( PSRAM_ENABLE, false );

  SphereRef sphereRef = findCachedSphere( sphereCache, { uint16_t(diameter), { indexedlight[0], indexedlight[1], indexedlight[2] } } );

  sphereIndex = sphereRef.index;

  if( sphereIndex >= 0 ) {
    renderCachedSphere( sprite, sphereRef, posx - radius, posy - radius, color );
    return;
  }

  float variablelight[3] = { float(indexedlight[0]), float(indexedlight[1]), float( indexedlight[2] ) };
  normalize(variablelight);

  int rstart = floor(-radius);
  int rend   = ceil(radius);

  int r2start = floor( -2 * radius );
  int r2end   = ceil(2 * radius);

  int span = rend - rstart;

  uint16_t mapwidth = span*(span);
  uint8_t* intensitymap = ( uint8_t*) calloc(mapwidth, sizeof(uint8_t));

  float minx = 512, maxx = -512;
  float miny = 512, maxy = -512;

  PixelColors pixels;

  for (i = rstart; i <= rend; i++) {
    x = i + .5;
    float xpow = x * x;
    for (j = r2start; j <= r2end; j++) {
      y = j / 2. + .5;
      float ypow = y * y;
      if (xpow + ypow <= radiuspow) {
        vec[0] = x;
        vec[1] = y;
        vec[2] = sqrt(radiuspow - xpow - ypow);
        normalize(vec);
        // TODO: Lambert this
        bvar   = pow(dot(variablelight, vec), k) + ambient/2;
        bfixed = pow(dot(fixedlight, vec), k) + ambient/2;

        intensity = (bfixed+bvar) * 127;
        if (intensity < 0) intensity = 0;
        if (intensity > 255)
          intensity = 255;

        //float angle = acos( ((x1-cx)*(x2-cx) + (y1-cy)*(y2-cy) + (z1-cz)*(z2-cz)) / (rr*rr) )

        int32_t mappos = int32_t(radius+x) + int32_t(radius+y) * span;

        if(mappos < 0 || mappos >= mapwidth  ) {
          Serial.printf("negative or overflow mappos: %d\n", mappos );
        }

        intensitymap[mappos] = intensity;

        if( radius+x < minx ) minx = radius+x;
        if( radius+y < miny ) miny = radius+y;
        if( radius+x > maxx ) maxx = radius+x;
        if( radius+y > maxy ) maxy = radius+y;
      }
    }
  }

  int16_t bwidth =  maxx+1-minx;
  int16_t bheight = maxy+1-miny;

  SphereCache sphere = { bwidth, bheight, uint16_t(diameter), int16_t(span-int(radius*2.)+1), pixels, { indexedlight[0], indexedlight[1], indexedlight[2] } };
  //Serial.printf("     spriteOffsets [%.2f, %.2f ~ %.2f, %.2f] span: %d\n", minx, miny, maxx, maxy, span);

  for( int32_t i=0; i<mapwidth; i++ ) {
    if( i>0 && i%span>=bwidth ) continue;
    if( i>0 && int(round(i/bwidth))%span>bheight ) continue;
    sphere.pixels.push_back( intensitymap[i] );
  }

  sphereCache.push_back( sphere );
  free( intensitymap );

  sphereRef.index = sphereCache.size()-1;
  renderCachedSphere( sprite, sphereRef, posx - radius, posy - radius, color );

}
