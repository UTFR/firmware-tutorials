// Direct C port of the original "race_condition_ex.cpp" handout: the minimal,
// hardware-agnostic illustration of a race condition. It never touched
// hardware even in the original Teensy version, so it's compiled here as a
// standalone object library, proved to build, but never linked into the
// LECTURE2_CONCURRENCY executable (thread1/thread2 are never actually called
// on real hardware -- this file is illustration only, same as before).
#define NUM_ITERS 1000
int x = 0;

void thread1(void) {
  for (int i = 0; i < NUM_ITERS; i++) { x += 1; }
}

void thread2(void) {
  for (int i = 0; i < NUM_ITERS; i++) { x *= 2; }
}
