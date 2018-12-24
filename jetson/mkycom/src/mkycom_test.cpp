#include "mkycom.h"

#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  int status = mkycom_init();
  if (status < 0) {
    cout << "failed to init com library, check connection." << endl;
    return -1;
  }

  ssize_t len;
  char buf[8];
  while (1) {
    len = mkycom_receive(buf, 7);
    cout << "len = " << len << endl;
    if (len < 7) continue;
    buf[len] = 0;
    cout << "received: " << buf << endl;
  }
  cout << "test completed..." << endl;
  return 0;
}

