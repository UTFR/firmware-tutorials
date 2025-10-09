#define NUM_ITERS 1000
int x = 0;

void thread1(void) {
  for(int i = 0; i < NUM_ITERS; i++) {
    x += 1;
  }
}

void thread2(void) {
  for(int i = 0; i < NUM_ITERS; i++) {
    x *= 2;
  }
}
