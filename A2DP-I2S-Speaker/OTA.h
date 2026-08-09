// AVRC Title key to enter OTA mode - comment out to remove OTA support
#define OTA_KEY "dHVuZWluc3RhbmNlc2hhZGV2YWx1ZXdvcmtlcmFmdGVyZmFt"

#ifdef OTA_KEY

class OTA_Driver {
public:
  OTA_Driver() {}
  void init(const char * ota_name = "BLAudio-OTA");
  void (*getMetaHandler())(uint8_t, const char *, size_t);
  void setMetaHandler(void (*cb)(uint8_t, const char *, size_t));
};

extern OTA_Driver OTA;

#endif
