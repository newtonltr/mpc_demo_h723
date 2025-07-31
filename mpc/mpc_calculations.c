/*
 * MPC在线计算函数实现
 * 此文件由osqp_genarate.py自动生成
 * 实现STM32上q, l, u向量的实时计算
 */

#include "mpc_matrices.h"
#include <string.h>

void calculate_q_vector(OSQPFloat* q_new, const OSQPFloat* x_current, const OSQPFloat* r_target) {
    // 计算 q = B_bar^T * Q_bar * (A_bar * x_current - r_stacked)
    
    // 步骤1: 计算A_bar * x_current
    OSQPFloat Ax[MPC_N * MPC_NX];
    for (OSQPInt i = 0; i < MPC_N * MPC_NX; i++) {
        Ax[i] = 0.0f;
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            Ax[i] += A_bar_data[i * MPC_NX + j] * x_current[j];
        }
    }
    
    // 步骤2: 计算A_bar * x_current - r_stacked
    OSQPFloat error_stacked[MPC_N * MPC_NX];
    for (OSQPInt i = 0; i < MPC_N; i++) {
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            error_stacked[i * MPC_NX + j] = Ax[i * MPC_NX + j] - r_target[j];
        }
    }
    
    // 步骤3: 计算q = BT_Q * error_stacked
    for (OSQPInt i = 0; i < MPC_N * MPC_NU; i++) {
        q_new[i] = 0.0f;
        for (OSQPInt j = 0; j < MPC_N * MPC_NX; j++) {
            q_new[i] += BT_Q_data[i * MPC_N * MPC_NX + j] * error_stacked[j];
        }
    }
}

void calculate_constraint_bounds(OSQPFloat* l_new, OSQPFloat* u_new, const OSQPFloat* x_current) {
    // 计算约束边界
    // l = [x_min_stacked - A_bar * x_current; u_min_stacked]
    // u = [x_max_stacked - A_bar * x_current; u_max_stacked]
    
    // 步骤1: 计算A_bar * x_current (重用上面的代码逻辑)
    OSQPFloat Ax[MPC_N * MPC_NX];
    for (OSQPInt i = 0; i < MPC_N * MPC_NX; i++) {
        Ax[i] = 0.0f;
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            Ax[i] += A_bar_data[i * MPC_NX + j] * x_current[j];
        }
    }
    
    // 步骤2: 设置状态约束边界
    for (OSQPInt i = 0; i < MPC_N; i++) {
        for (OSQPInt j = 0; j < MPC_NX; j++) {
            OSQPInt idx = i * MPC_NX + j;
            l_new[idx] = x_min_single[j] - Ax[idx];
            u_new[idx] = x_max_single[j] - Ax[idx];
        }
    }
    
    // 步骤3: 设置输入约束边界
    for (OSQPInt i = 0; i < MPC_N; i++) {
        for (OSQPInt j = 0; j < MPC_NU; j++) {
            OSQPInt idx = MPC_N * MPC_NX + i * MPC_NU + j;
            l_new[idx] = u_min_single[j];
            u_new[idx] = u_max_single[j];
        }
    }
}
