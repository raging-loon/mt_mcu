#ifndef MT_MCU_UTILS_H_
#define MT_MCU_UTILS_H_

#define MT_CLEAR_FLAG(x, value) x &= ~(value)

#define MT_HAS_FLAG(x, flag) ((x & flag) == flag)
#endif // MT_MCU_UTILS_H_