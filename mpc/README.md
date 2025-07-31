# 四轮八驱（4W8D）转向系统模型预测控制

基于OSQP的高效嵌入式平台实现

## 项目简介

本项目实现了四轮八驱（4W8D）移动平台转向系统的模型预测控制（MPC）解决方案。相比传统的开环比例控制或闭环PID控制策略，MPC采用集中式多变量优化方法，能够从根本上解决四个转向轮同步性问题，满足高精度高协同的运动控制需求。

### 核心优势

- **集中式协调控制**：一个统一的优化框架同时计算四个转向轮的最优控制输入，内在保证运动同步性
- **高效嵌入式实现**：基于OSQP求解器的代码生成功能，无动态内存分配，无外部库依赖
- **实时性能优化**：利用硬件FPU加速，支持毫秒级控制周期
- **约束处理能力**：显式处理角度和角速度物理限制，保证系统安全运行

## 技术方案

### 系统建模
- 建立适用于控制设计的状态空间模型
- 每个转向执行机构建模为离散时间积分器
- 状态向量：四个转向轮角度 `[θ1, θ2, θ3, θ4]`
- 控制输入：四个目标角速度 `[ω1, ω2, ω3, ω4]`

### MPC控制器
- 将转向控制问题构建为标准二次规划（QP）优化问题
- 二次代价函数平衡跟踪性能和控制平顺性
- 显式约束处理：角度限制和速度限制
- 滚动时域优化策略

### OSQP求解器
- 基于交替方向乘子法（ADMM）的高效QP求解
- 代码生成功能：预编译求解器核心，静态内存分配
- 嵌入式优化：无除法运算，数值稳定性强
- 热启动机制：利用前一时刻解加速收敛

## 项目结构

```
4w8dmpc/
├── osqp_genarate.py                    # OSQP代码生成脚本
├── mpc_solver_4w8d/                    # 生成的OSQP求解器
│   ├── workspace.c                     # 预编译求解器核心 (91KB)
│   ├── workspace.h                     # 全局solver实例声明
│   ├── mpc_matrices.h                  # 预计算矩阵常量 (28KB)
│   ├── mpc_calculations.c              # 实时计算函数 (2.3KB)
│   ├── inc/public/                     # OSQP公共API头文件
│   │   ├── osqp.h                      # 主要API入口
│   │   ├── osqp_api_types.h            # 核心数据结构
│   │   └── osqp_api_functions.h        # API函数声明
│   └── src/                            # OSQP实现源码
├── mpc_solver_4w8d_lite/               # 手写梯度下降实现（验证对比）
│   ├── mpc_4w8d_lite.c                 # 轻量级MPC实现
│   ├── mpc_4w8d_lite.h                 # 头文件定义
│   └── test_mpc.c                      # 测试程序
└── 四轮八驱（4W8D）转向系统模型预测控制.md  # 详细技术文档
```

## 性能对比

经过测试验证，基于OSQP的ADMM算法配合CSR稀疏矩阵压缩方法相比手写梯度下降优化在计算性能上有显著优势：

- **OSQP求解器**：高效ADMM算法，预计算KKT矩阵分解
- **梯度下降方法**：传统优化算法，每次迭代重新计算梯度
- **性能差距**：OSQP速度比梯度下降快约**4倍**

## 快速开始

### 1. 代码生成

```bash
# 安装依赖
pip install osqp numpy scipy

# 运行代码生成脚本
python osqp_genarate.py
```

### 2. STM32集成

1. **项目配置**
   - 将`mpc_solver_4w8d`文件夹复制到STM32项目中
   - 配置包含路径：`../Core/MPC_Solver/inc/public`
   - 添加预处理器宏：`OSQP_EMBEDDED_MODE=1`

2. **FPU优化**
   - 编译器设置：Floating-point unit = FPv4-SP-D16
   - Floating-point ABI = Hard
   - 运行时启用FPU（通常在system_stm32xxxx.c中已包含）

3. **实时控制循环**
```c
#include "osqp.h"
#include "workspace.h"
#include "mpc_matrices.h"

void mpc_control_loop(void) {
    // 1. 在线参数计算
    calculate_q_vector(q_new, x_current, r_target);
    calculate_constraint_bounds(l_new, u_new, x_current);
    
    // 2. 更新求解器参数
    osqp_update_data_vec(&solver, q_new, l_new, u_new);
    
    // 3. 求解优化问题
    OSQPInt exit_code = osqp_solve(&solver);
    
    // 4. 执行控制决策
    if (exit_code == 0 && solver.info->status_val == OSQP_SOLVED) {
        memcpy(u_optimal, solver.solution->x, MPC_NU * sizeof(OSQPFloat));
        // 发送速度指令到伺服驱动器
        send_can_velocities(u_optimal);
        // 热启动优化
        osqp_warm_start(&solver, solver.solution->x, solver.solution->y);
    }
}
```

## 参数调优

### MPC参数
- **预测时域 N**：建议10-20，平衡性能和计算量
- **状态权重矩阵 Q**：控制跟踪性能，增大对应元素可提高该轮响应速度
- **输入权重矩阵 R**：控制输入平滑度，增大可减少控制输出颠簸

### 性能目标
- 总执行时间 < 10ms（100Hz控制频率）
- 迭代次数 < 20（热启动后 < 10）
- 残差 < 1e-3，确保求解质量

## 通信接口

系统通过CANopen协议与伺服驱动器通信：
- **控制对象**：0x60FF（目标速度）
- **反馈对象**：0x6064（位置实际值）、0x606C（速度实际值）
- **通信频率**：与MPC控制周期同步（建议100Hz）

## 技术文档

详细的理论推导、数学建模、实现细节和调优指南请参考：
[四轮八驱（4W8D）转向系统模型预测控制：基于OSQP的高效嵌入式平台实现.md](./四轮八驱（4W8D）转向系统模型预测控制：基于OSQP的高效嵌入式平台实现.md)

## 许可证

本项目基于研究和教育目的开发，请在使用时注明出处。

## 参考文献

1. OSQP: Operator Splitting Quadratic Program Solver
2. Embedded Code Generation Using the OSQP Solver - Stanford University
3. CANopen Application Layer and Communication Profile
4. STM32 FPU Programming Manual