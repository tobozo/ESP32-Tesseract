#pragma once

struct FrameSrc
{
  const uint8_t *data;
  size_t   data_len;
};

struct FrameDimensions
{
  uint16_t width;
  uint16_t height;
};


enum AnimationImgType
{
  IMG_RAW,
  IMG_JPG,
  IMG_PNG,
  IMG_QOI
};


struct Animation
{
  FrameSrc *frames;
  FrameDimensions dimensions;
  size_t framesCount;
  AnimationImgType type;

  uint16_t width()
  {
    return dimensions.width;
  }

  uint16_t height()
  {
    return dimensions.height;
  }

  FrameSrc frame(uint16_t framenum)
  {
    return frames[framenum];
  }

};
