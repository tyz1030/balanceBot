#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_structs.h"
#include <rc/math/filter.h>
#define CFG_PATH "bin/pid.cfg"

int mb_controller_init(rc_filter_t**, pid_gains_t *);
int mb_controller_load_config(pid_gains_t *);
int mb_controller_update(mb_state_t *, mb_setpoints_t *, rc_filter_t**);
int mb_controller_cleanup();

#endif
