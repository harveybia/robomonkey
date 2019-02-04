#include "mkycom.h"

#include <pthread.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  int status = mkycom_init();
  if (status < 0) {
    cout << "failed to init com library, check connection." << endl;
    return -1;
  }

  // ssize_t len;
  // char buf[8];
  // while (1) {
  //   len = mkycom_receive(buf, 7);
  //   cout << "len = " << len << endl;
  //   if (len < 7) continue;
  //   buf[len] = 0;
  //   cout << "received: " << buf << endl;
  // }
  // cout << "test completed..." << endl;

  fprintf(stderr, "Size of chassis_information: %lu", sizeof(chassis_info_t));

  pthread_t comm_thread;
  fprintf(stderr, "Before thread\n");

  pthread_create(&comm_thread, NULL, read_and_unpack_thread, NULL);

  fprintf(stderr, "Thread started\n");
  
  pthread_join(comm_thread, NULL);

  fprintf(stderr, "After thread\n");

  return 0;
}

