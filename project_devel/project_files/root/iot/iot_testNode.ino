#ifdef TESTING
#include <stoopidHome.h>

void setup()
{
  
    Serial.begin(9600);
    while (!Serial)  ;  
  
}

void loop()
{
    TestRunner::run();

}
#endif 