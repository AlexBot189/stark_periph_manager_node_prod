/*
 * stark_state_machine.cpp — 4 状态机实现
 * Copyright (c) 2026 zhiqiang.yang
 *
 * enter/exit 钩子为日志+状态记录, 业务逻辑在 main.cpp 中驱动.
 * 校准逻辑已移至 main.cpp 主循环, 不再作为独立状态.
 */
#include "motor/motor_state.h"
#include "motor/motor_context.h"
#include <log_helper/LogHelper.h>

stark_state_t g_stark_state = STATE_BOOTING;

const char* state_name(stark_state_t s)
{
    switch (s) {
    case STATE_BOOTING: return "BOOTING";
    case STATE_READY:   return "READY";
    case STATE_RUNNING: return "RUNNING";
    case STATE_FAULT:   return "FAULT";
    default:            return "UNKNOWN";
    }
}

/* enter */

void enter_booting(void) {
    ECO_INFO_NEW("[StateMachine] enter BOOTING");
}

void enter_ready(void) {
    ECO_INFO_NEW("[StateMachine] enter READY — ready, waiting algorithm");
}

void enter_running(void) {
    ECO_INFO_NEW("[StateMachine] enter RUNNING");
}

void enter_fault(void) {
    ECO_INFO_NEW("[StateMachine] enter FAULT — emergency stop");
}

/* exit */

void exit_booting(void) {
    ECO_INFO_NEW("[StateMachine] exit BOOTING");
}

void exit_ready(void) {}

void exit_running(void) {
    ECO_WARN_NEW("[StateMachine] exit RUNNING — control suspended");
}

void exit_fault(void) {
    ECO_INFO_NEW("[StateMachine] exit FAULT — attempting recovery");
}

/* 钩子表 */

enter_fn enter_hooks[STARK_STATE_COUNT] = {
    enter_booting, enter_ready, enter_running, enter_fault,
};

exit_fn exit_hooks[STARK_STATE_COUNT] = {
    exit_booting, exit_ready, exit_running, exit_fault,
};

/* state_transition_allowed */

bool state_transition_allowed(stark_state_t from, stark_state_t to)
{
    if (from == to) return true;

    /* 允许 FAULT 从任意状态触发 */
    if (to == STATE_FAULT) return true;

    /* 顺序: BOOTING ,  READY ,  RUNNING */
    if (from == STATE_BOOTING && to == STATE_READY)   return true;
    if (from == STATE_READY   && to == STATE_RUNNING) return true;

    /* 校准取消/重校准: RUNNING ,  READY */
    if (from == STATE_RUNNING && to == STATE_READY) return true;

    /* RUNNING ,  FAULT (已在上面 FAULT 通用规则中覆盖) */

    /* FAULT ,  READY (恢复) */
    if (from == STATE_FAULT && to == STATE_READY) return true;

    /* READY ,  FAULT (硬件异常) */
    if (from == STATE_READY && to == STATE_FAULT) return true;

    return false;
}

/* state_transition */

bool state_transition(stark_state_t to)
{
    stark_state_t from = g_stark_state;

    if (!state_transition_allowed(from, to)) {
        ECO_WARN_NEW("[StateMachine] denied: {} ,  {}",
                     state_name(from), state_name(to));
        return false;
    }

    if (exit_hooks[from]) exit_hooks[from]();
    g_stark_state = to;
    if (enter_hooks[to]) enter_hooks[to]();

    ECO_INFO_NEW("[StateMachine] {} ,  {}", state_name(from), state_name(to));
    return true;
}
