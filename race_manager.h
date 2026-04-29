#ifndef RACE_MANAGER_H
#define RACE_MANAGER_H

#include "src/gui_guider.h"

#ifdef __cplusplus
extern "C" {
#endif

void race_manager_init(lv_ui *ui);
void race_manager_enter(void);
void race_manager_exit(void);
bool race_manager_is_active(void);
void race_manager_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
