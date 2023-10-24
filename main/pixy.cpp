#include <Pixy2.h>
#include "AllenFunctions.h"

void setupPixy() {
  pixy.init();
}

Block getPixyBlockData(){ 
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks){
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    pixy.ccc.blocks[0].print();
    return pixy.ccc.blocks[0];
  }  
}

uint16_t getPixyXCoord() {
  return getPixyBlockData().m_x;
}

uint16_t getPixyColor() {
  return getPixyBlockData().m_signature;
}



/* main.ino - all pixy module together in one file -> can compile
 * when separate -> can't compile, receive errors


#include "AllenFunctions.h"
// ----- AllenFunctions.h file -----
#include <Pixy2.h>
Pixy2 pixy;
void setupPixy();
Block getPixyBlockData();
uint16_t getPixyXCoord();
uint16_t getPixyColor();

// --------- main.ino file ---------
void setup(){
  Serial.begin(9600);
  setupPixy();
}

void loop(){ 
  Block pixyBlocks = getPixyBlockData();
  bool targetIsOnTheLeft = getPixyXCoord() < 153; // 0-315
  bool targetIsOnTheRight = getPixyXCoord() > 162;
  
  if (targetIsOnTheLeft) {
    //turn left how long
  }
  else if (targetIsOnTheRight) {
    //turn right
  }
  else {
    //don't turn
  }
  
}

// ----- pixy.cpp file -----
void setupPixy() {
  pixy.init();
}

Block getPixyBlockData(){ 
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks){
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    pixy.ccc.blocks[0].print();
    return pixy.ccc.blocks[0];
  }  
}

uint16_t getPixyXCoord() {
  return getPixyBlockData().m_x;
}

uint16_t getPixyColor() {
  return getPixyBlockData().m_signature;
}

 */
