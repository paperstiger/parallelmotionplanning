#ifndef NAOCUP_H
#define NAOCUP_H

#ifdef __cplusplus
extern "C" {
#endif
#include "prrts.h"

prrts_system_t *naocup_create_system();
void naocup_free_system(prrts_system_t *system);

int naocup_main(int argc, char *argv[]);
#ifdef __cplusplus
}
#endif
#endif /* NAOCUP_H */
