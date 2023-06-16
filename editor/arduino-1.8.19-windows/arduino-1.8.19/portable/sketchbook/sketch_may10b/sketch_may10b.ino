#include "FastRCReader.h"

//Time definitions
#define REFRESHRATE 10

//Port definitions
#define FROMCHANNELPIN 0
#define TOCHANNELPIN 1

RCChannelMapper RC;

void setup() {
  Serial.begin(115200);
  RC.begin();

  RC.recvTimeout(25000);

  //Add/Activate all needed channels
  for (uint8_t i = FROMCHANNELPIN; i <= TOCHANNELPIN; i++) {
    RC.addChannel(i);
    RC.setMap(1000, 2000, i);
  }

}

void loop() {
  //Plot all Channels to the serial Plotter
  for (uint8_t i = FROMCHANNELPIN; i <= TOCHANNELPIN; i++) {
    Serial.print(RC.getChannel(i)); Serial.print("\t");
  } Serial.println(RC.isRecvOn());

  //Wait till the refreshrate is expiered
  unsigned long waitSince = millis();
  while ((millis() - waitSince) < REFRESHRATE);
}
