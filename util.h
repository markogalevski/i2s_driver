#ifndef _UTIL_H
#define _UTIL_H

#define CRITICAL(expr) \
  __disable_irq(); \
  expr; \
  __enable_irq();


#endif /*UTIL_H*/
