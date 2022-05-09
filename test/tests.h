
#include <AUnit.h>

test(correct) {
  int x = 1;
  assertEqual(x, 1);
}

test(incorrect) {
  int x = 2;
  assertNotEqual(x, 1);
}