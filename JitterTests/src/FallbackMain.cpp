#include "TestSupport.hpp"

#if !JITTER_USE_CATCH2
int main()
{
    return JitterTests::RunAll();
}
#endif
