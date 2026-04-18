#ifndef __VIBE_H__
#define __VIBE_H__

#include <stddef.h>
#include <stdint.h>
#include "Audio.h"

class Vibe_Driver {
public:
  Vibe_Driver() {}
  void (*getMetaHandler())(uint8_t, const char *, size_t);
  void setMetaHandler(void (*cb)(uint8_t, const char *, size_t));
  void setBiquadTarget(Audio_Codec &target);
  void clearBiquadTarget();
};

extern Vibe_Driver Vibe;

#endif
