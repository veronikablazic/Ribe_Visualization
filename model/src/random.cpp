#include <thread>
#include <atomic>
#include <malloc.h>
#include "random.hpp"

static std::atomic<unsigned long> rnd_eng_seed = rnd_eng_type::default_seed;

void rnd_seed(unsigned long seed)
{
  rnd_eng_seed = seed;
}

rnd_eng_type& rnd_eng()
{
  static __declspec (thread) rnd_eng_type* tls = nullptr;
  if (nullptr == tls) 
  {
    void* raw = _aligned_malloc(sizeof(rnd_eng_type), 64);  // freed by OS at exit.
    tls = new (raw) rnd_eng_type(rnd_eng_seed.fetch_add(11));
  }
  return *tls;
}