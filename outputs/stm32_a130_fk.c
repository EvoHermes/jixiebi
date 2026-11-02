#include "stm32_a130_fk.h"
#include <math.h>

#define A130_A2  0.105f
#define A130_A3  0.0975f
#define A130_A4  0.02822f
#define A130_D1  0.1085f
#define A130_D5  0.17283f

void A130_FK(float j1, float j2, float j3, float j4, float j5,
             float *x, float *y, float *z)
{
    float t23 = j2 + j3;
    float t234 = t23 + j4;
    float reach;

    (void)j5; /* J5 changes wrist orientation, not this position formula. */

    reach = A130_A3 * sin(t23)
          + A130_A2 * sin(j2)
          + A130_A4 * cos(t234)
          + A130_D5 * sin(t234);

    *x = -cos(j1) * reach;
    *y = -sin(j1) * reach;
    *z = A130_D1
       + A130_A3 * cos(t23)
       + A130_A2 * cos(j2)
       + A130_D5 * cos(t234)
       - A130_A4 * sin(t234);
}
