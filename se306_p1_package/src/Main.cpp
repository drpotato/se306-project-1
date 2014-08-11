#include "R0.h"

int main(int argc, char **argv)
{
  Actor* r0 = new R0();

  while(1) {    
    r0->execute();
  }
}