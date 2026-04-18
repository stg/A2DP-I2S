#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include <stddef.h>

typedef enum {
  RES_MP3_LOGO,
  RES_MP3_CONNECT,
  RES_MP3_DISCONNECT,
} res_id;

// Resource arrays (defined in Resources.cpp)
extern const unsigned char logo_mp3[23808];
extern const unsigned char connect_mp3[6336];
extern const unsigned char disconnect_mp3[6336];

// Accessors
const unsigned char *resourceData(res_id id);
size_t resourceSize(res_id id);

#endif // __RESOURCES_H__
