/*
 * stark_state_machine.h — 4 状态机
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 状态: BOOTING ,  READY ,  RUNNING
 *       任意状态 ,  FAULT, FAULT ,  READY
 *
 * 使用: state_transition(new_state) 一次调用完成 exit+enter
 */
#pragma once

#include "stark_shm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*enter_fn)(void);
typedef void (*exit_fn)(void);

extern stark_state_t g_stark_state;

const char* state_name(stark_state_t s);

extern void enter_booting(void);
extern void enter_ready(void);
extern void enter_running(void);
extern void enter_fault(void);

extern void exit_booting(void);
extern void exit_ready(void);
extern void exit_running(void);
extern void exit_fault(void);

extern enter_fn enter_hooks[STARK_STATE_COUNT];
extern exit_fn  exit_hooks[STARK_STATE_COUNT];

bool state_transition_allowed(stark_state_t from, stark_state_t to);
bool state_transition(stark_state_t to);

#ifdef __cplusplus
}
#endif
