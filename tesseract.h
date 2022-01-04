#pragma once

#include "lookup_tables.h"

typedef std::vector<std::vector<float>> PointsArray;
typedef std::array<float,4> Coords;
typedef std::array<uint8_t,4> ByteCoords;

typedef std::vector<std::array<int16_t,2>> LinesIndexesArray;
typedef std::vector<int16_t> PointsIndexesArray;
typedef std::array<int16_t,4> Coords4D;
typedef std::array<int16_t,3> Coords3D;

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
static int framesCount = 60; // for the images animation, dynamically replaced

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
static bool drawfps = true;

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

static PointsArray rot3d =
{
  { 1, 0, 0 },
  { 0, (float)romcos(QuarterPI), (float)-romsin(QuarterPI) },
  { 0, (float)romsin(QuarterPI), (float)romcos(QuarterPI) }
};


static bool pointIsIndexed( int16_t point )
{
  for(byte i=0;i<points4D.size();i++) {
    if( point == points4D[i] ) {
      return true;
    }
  }
  return false;
}


static void pointPush( int16_t point )
{
  if( ! pointIsIndexed( point ) ) {
    points4D.push_back( point );
  }
}


static bool lineIsIndexed( std::array<int16_t,2> line )
{
  for(byte i=0;i<lines4D.size();i++) {
    if( (line[0] == lines4D[i][0] && line[1] == lines4D[i][1])
     || (line[1] == lines4D[i][0] && line[0] == lines4D[i][1])
    ) {
      return true;
    }
  }
  return false;
}


static void linePush( std::array<int16_t,2> line )
{
  if( !lineIsIndexed( line ) ) {
    lines4D.push_back( line );
  }
}


/* sort xyz by z depth */
static void zSortPoints()
{
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


static void setCoordsCache( ByteCoords cc, Coords &pout )
{
  byte x = cc[0], y=cc[1], z=cc[2], w=cc[3];
  int16_t cacheID = x + y*2 + z*4 + w*8;
  cache4D[cacheID] = { (int16_t)pout[0], (int16_t)pout[1], (int16_t)pout[2], (int16_t)pout[3] };
}


static bool isInCoordsCache( ByteCoords cc )
{
  byte x = cc[0], y=cc[1], z=cc[2], w=cc[3];
  int16_t cacheID = x + y*2 + z*4 + w*8;
  if( cache4D[cacheID][0] + cache4D[cacheID][1] + cache4D[cacheID][2] + cache4D[cacheID][3] == 0 ) {
    return false;
  }
  return true;
}


static void clearCoordsCache()
{
  for( byte  i=0;i<16;i++ ) {
    cache4D[i] = {0,0,0,0};
  }
}


static void multiplyCoords( PointsArray &A, PointsArray &B, PointsArray &C)
{
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


static void multiplyCoords( PointsArray &A, Coords &B, PointsArray &C)
{
  PointsArray CoordsToPointsArray(4);
  for(byte i=0;i<B.size();i++) {
    CoordsToPointsArray[i].push_back(B[i]);
  }
  multiplyCoords( A, CoordsToPointsArray, C );
}


static void transformPoint(ByteCoords p0)
{

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

