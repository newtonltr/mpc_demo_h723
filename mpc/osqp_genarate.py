import osqp  
import numpy as np  
from scipy import sparse
import warnings

# 抑制OSQP矩阵格式转换警告
warnings.filterwarnings('ignore', message='Converting sparse .* to a CSC matrix.*')

# --- 1. 定义MPC参数 (示例值) ---
N = 10  # 预测时域  
Ts = 0.01 # 采样时间 (100Hz)  
nx = 4  # 状态维度 (4个角度)  
nu = 4  # 输入维度 (4个角速度)

# 权重矩阵  
Q = sparse.diags([10.0, 10.0, 10.0, 10.0])  
R = sparse.diags([0.1, 0.1, 0.1, 0.1])  
QN = Q # 终端权重

# 约束  
theta_max = np.deg2rad(45.0)  
omega_max = np.deg2rad(90.0)  
u_min = -omega_max * np.ones(nu)  
u_max = omega_max * np.ones(nu)  
x_min = -theta_max * np.ones(nx)  
x_max = theta_max * np.ones(nx)

# --- 2. 构建QP矩阵 (离线计算) ---
# 状态空间模型  
A = sparse.eye(nx)  
B = sparse.eye(nx) * Ts

# 构建代价函数矩阵 P 和 q  
# P = B_bar.T * Q_bar * B_bar + R_bar  
# q = B_bar.T * Q_bar * (A_bar * x_k - r_bar)  
# 这里只构建 P 和 q 的不变部分  
A_bar = sparse.vstack([A**i for i in range(1, N + 1)])  

# 构建MPC预测时域下的输入块矩阵B_bar（下三角块带状结构）
# B_bar是一个(N*nx, N*nu)的矩阵，具有下三角块结构
B_bar_blocks = []
for i in range(N):
    row_blocks = []
    for j in range(N):
        if i >= j:
            # 计算A^(i-j) @ B
            power_A = A**(i-j)
            block = power_A @ B
            row_blocks.append(block)
        else:
            # 上三角部分用零矩阵填充
            row_blocks.append(sparse.csr_matrix((nx, nu)))
    B_bar_blocks.append(row_blocks)

B_bar = sparse.bmat(B_bar_blocks)

Q_bar = sparse.block_diag([Q] * (N - 1) + [QN])  
R_bar = sparse.block_diag([R] * N)

P = B_bar.T @ Q_bar @ B_bar + R_bar  
# 转换为CSC格式以优化OSQP性能
P = P.tocsc()
# q 的计算依赖于 x_k 和 r_k，这里先用0填充  
q_dummy = np.zeros(nu * N)

# 构建约束矩阵 Ac, l, u  
# 状态约束: x_min <= A_bar * x_k + B_bar * z <= x_max  
# 输入约束: u_min <= z <= u_max  
A_c = sparse.vstack([
    B_bar,                    # 状态约束矩阵部分 (N*nx, N*nu)
    sparse.eye(nu * N)        # 输入约束矩阵部分 (N*nu, N*nu)
])
# 转换为CSC格式以优化OSQP性能
A_c = A_c.tocsc()
# l 和 u 的计算也依赖 x_k, 先用-inf和+inf填充  
l_dummy = np.hstack([np.full(nx * N, -np.inf), np.tile(u_min, N)])  
u_dummy = np.hstack([np.full(nx * N, np.inf), np.tile(u_max, N)])

# --- 矩阵维度验证 ---
print("=== MPC QP 矩阵维度验证 ===")
print(f"预测时域 N = {N}, 状态维度 nx = {nx}, 输入维度 nu = {nu}")
print(f"A_bar shape: {A_bar.shape} (应为 {N*nx} x {nx})")
print(f"B_bar shape: {B_bar.shape} (应为 {N*nx} x {N*nu})")
print(f"Q_bar shape: {Q_bar.shape} (应为 {N*nx} x {N*nx})")
print(f"R_bar shape: {R_bar.shape} (应为 {N*nu} x {N*nu})")
print(f"P shape: {P.shape} (应为 {N*nu} x {N*nu}), format: {P.format}, nnz: {P.nnz}")
print(f"A_c shape: {A_c.shape} (应为 {N*nx + N*nu} x {N*nu}), format: {A_c.format}, nnz: {A_c.nnz}")
print(f"q_dummy length: {len(q_dummy)} (应为 {N*nu})")
print(f"l_dummy length: {len(l_dummy)} (应为 {N*nx + N*nu})")
print(f"u_dummy length: {len(u_dummy)} (应为 {N*nx + N*nu})")
print("=========================")

# --- 3. 设置OSQP问题 ---
prob = osqp.OSQP()  
# 使用占位符数据进行设置  
prob.setup(P=P, q=q_dummy, A=A_c, l=l_dummy, u=u_dummy, verbose=False)

# --- 4. 生成C代码 ---
# 'mpc_solver_4w8d' 是您想存放生成代码的文件夹名  
# parameters='vectors' 告诉代码生成器 P 和 A 是固定的，q, l, u 是可变的  
prob.codegen('mpc_solver_4w8d',  
             parameters='vectors',  
             force_rewrite=True)

print("C code generation complete!")

# --- 5. 导出MPC计算矩阵到C头文件 ---
def export_matrices_to_c_header():
    """
    导出MPC计算所需的关键矩阵到C头文件
    这些矩阵用于在STM32上实时计算q, l, u向量
    """
    
    # 创建输出文件
    header_filename = 'mpc_solver_4w8d/mpc_matrices.h'
    
    with open(header_filename, 'w') as f:
        f.write("/*\n")
        f.write(" * MPC计算矩阵常量定义\n")
        f.write(" * 此文件由osqp_genarate.py自动生成\n")
        f.write(" * 包含STM32实时计算q, l, u向量所需的所有矩阵常量\n")
        f.write(" */\n\n")
        
        f.write("#ifndef MPC_MATRICES_H\n")
        f.write("#define MPC_MATRICES_H\n\n")
        
        f.write("#include \"osqp.h\"\n\n")
        
        # 导出MPC参数
        f.write("// MPC参数定义\n")
        f.write(f"#define MPC_N  {N}   // 预测时域\n")
        f.write(f"#define MPC_NX {nx}  // 状态维度\n")
        f.write(f"#define MPC_NU {nu}  // 输入维度\n")
        f.write(f"#define MPC_TS {Ts}f // 采样时间\n\n")
        
        # 导出A_bar矩阵（用于计算A_bar * x_current）
        f.write("// A_bar矩阵 (预测状态转移矩阵)\n")
        f.write("// 维度: (N*nx) x nx = 40 x 4\n")
        f.write("// 用于计算: A_bar * x_current\n")
        A_bar_dense = A_bar.toarray()
        f.write("static const OSQPFloat A_bar_data[MPC_N * MPC_NX * MPC_NX] = {\n")
        for i in range(A_bar_dense.shape[0]):
            f.write("    ")
            for j in range(A_bar_dense.shape[1]):
                f.write(f"{A_bar_dense[i,j]:.10f}f")
                if i < A_bar_dense.shape[0]-1 or j < A_bar_dense.shape[1]-1:
                    f.write(", ")
            f.write("\n")
        f.write("};\n\n")
        
        # 导出B_bar^T * Q_bar矩阵（用于计算q向量）
        BT_Q = (B_bar.T @ Q_bar).toarray()
        f.write("// B_bar^T * Q_bar矩阵 (用于计算q向量)\n")
        f.write("// q = BT_Q * (A_bar * x_current - r_stacked)\n")
        f.write("// 维度: (N*nu) x (N*nx) = 40 x 40\n")
        f.write("static const OSQPFloat BT_Q_data[MPC_N * MPC_NU * MPC_N * MPC_NX] = {\n")
        for i in range(BT_Q.shape[0]):
            f.write("    ")
            for j in range(BT_Q.shape[1]):
                f.write(f"{BT_Q[i,j]:.10f}f")
                if i < BT_Q.shape[0]-1 or j < BT_Q.shape[1]-1:
                    f.write(", ")
            f.write("\n")
        f.write("};\n\n")
        
        # 导出约束相关常量
        f.write("// 约束常量\n")
        f.write("static const OSQPFloat x_min_single[MPC_NX] = {")
        for i, val in enumerate(x_min):
            f.write(f"{val:.6f}f")
            if i < len(x_min) - 1:
                f.write(", ")
        f.write("};\n")
        
        f.write("static const OSQPFloat x_max_single[MPC_NX] = {")
        for i, val in enumerate(x_max):
            f.write(f"{val:.6f}f")
            if i < len(x_max) - 1:
                f.write(", ")
        f.write("};\n")
        
        f.write("static const OSQPFloat u_min_single[MPC_NU] = {")
        for i, val in enumerate(u_min):
            f.write(f"{val:.6f}f")
            if i < len(u_min) - 1:
                f.write(", ")
        f.write("};\n")
        
        f.write("static const OSQPFloat u_max_single[MPC_NU] = {")
        for i, val in enumerate(u_max):
            f.write(f"{val:.6f}f")
            if i < len(u_max) - 1:
                f.write(", ")
        f.write("};\n\n")
        
        # 导出计算函数声明
        f.write("// MPC在线计算函数声明\n")
        f.write("void calculate_q_vector(OSQPFloat* q_new, const OSQPFloat* x_current, const OSQPFloat* r_target);\n")
        f.write("void calculate_constraint_bounds(OSQPFloat* l_new, OSQPFloat* u_new, const OSQPFloat* x_current);\n\n")
        
        f.write("#endif // MPC_MATRICES_H\n")
    
    print(f"Matrix constants exported to: {header_filename}")

# --- 6. 生成MPC计算函数的C实现文件 ---
def generate_mpc_calculations_c():
    """
    生成MPC在线计算函数的C实现
    """
    
    c_filename = 'mpc_solver_4w8d/mpc_calculations.c'
    
    with open(c_filename, 'w') as f:
        f.write("/*\n")
        f.write(" * MPC在线计算函数实现\n")
        f.write(" * 此文件由osqp_genarate.py自动生成\n")
        f.write(" * 实现STM32上q, l, u向量的实时计算\n")
        f.write(" */\n\n")
        
        f.write("#include \"mpc_matrices.h\"\n")
        f.write("#include <string.h>\n\n")
        
        # q向量计算函数
        f.write("void calculate_q_vector(OSQPFloat* q_new, const OSQPFloat* x_current, const OSQPFloat* r_target) {\n")
        f.write("    // 计算 q = B_bar^T * Q_bar * (A_bar * x_current - r_stacked)\n")
        f.write("    \n")
        f.write("    // 步骤1: 计算A_bar * x_current\n")
        f.write("    OSQPFloat Ax[MPC_N * MPC_NX];\n")
        f.write("    for (OSQPInt i = 0; i < MPC_N * MPC_NX; i++) {\n")
        f.write("        Ax[i] = 0.0f;\n")
        f.write("        for (OSQPInt j = 0; j < MPC_NX; j++) {\n")
        f.write("            Ax[i] += A_bar_data[i * MPC_NX + j] * x_current[j];\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    \n")
        f.write("    // 步骤2: 计算A_bar * x_current - r_stacked\n")
        f.write("    OSQPFloat error_stacked[MPC_N * MPC_NX];\n")
        f.write("    for (OSQPInt i = 0; i < MPC_N; i++) {\n")
        f.write("        for (OSQPInt j = 0; j < MPC_NX; j++) {\n")
        f.write("            error_stacked[i * MPC_NX + j] = Ax[i * MPC_NX + j] - r_target[j];\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    \n")
        f.write("    // 步骤3: 计算q = BT_Q * error_stacked\n")
        f.write("    for (OSQPInt i = 0; i < MPC_N * MPC_NU; i++) {\n")
        f.write("        q_new[i] = 0.0f;\n")
        f.write("        for (OSQPInt j = 0; j < MPC_N * MPC_NX; j++) {\n")
        f.write("            q_new[i] += BT_Q_data[i * MPC_N * MPC_NX + j] * error_stacked[j];\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("}\n\n")
        
        # 约束边界计算函数
        f.write("void calculate_constraint_bounds(OSQPFloat* l_new, OSQPFloat* u_new, const OSQPFloat* x_current) {\n")
        f.write("    // 计算约束边界\n")
        f.write("    // l = [x_min_stacked - A_bar * x_current; u_min_stacked]\n")
        f.write("    // u = [x_max_stacked - A_bar * x_current; u_max_stacked]\n")
        f.write("    \n")
        f.write("    // 步骤1: 计算A_bar * x_current (重用上面的代码逻辑)\n")
        f.write("    OSQPFloat Ax[MPC_N * MPC_NX];\n")
        f.write("    for (OSQPInt i = 0; i < MPC_N * MPC_NX; i++) {\n")
        f.write("        Ax[i] = 0.0f;\n")
        f.write("        for (OSQPInt j = 0; j < MPC_NX; j++) {\n")
        f.write("            Ax[i] += A_bar_data[i * MPC_NX + j] * x_current[j];\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    \n")
        f.write("    // 步骤2: 设置状态约束边界\n")
        f.write("    for (OSQPInt i = 0; i < MPC_N; i++) {\n")
        f.write("        for (OSQPInt j = 0; j < MPC_NX; j++) {\n")
        f.write("            OSQPInt idx = i * MPC_NX + j;\n")
        f.write("            l_new[idx] = x_min_single[j] - Ax[idx];\n")
        f.write("            u_new[idx] = x_max_single[j] - Ax[idx];\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("    \n")
        f.write("    // 步骤3: 设置输入约束边界\n")
        f.write("    for (OSQPInt i = 0; i < MPC_N; i++) {\n")
        f.write("        for (OSQPInt j = 0; j < MPC_NU; j++) {\n")
        f.write("            OSQPInt idx = MPC_N * MPC_NX + i * MPC_NU + j;\n")
        f.write("            l_new[idx] = u_min_single[j];\n")
        f.write("            u_new[idx] = u_max_single[j];\n")
        f.write("        }\n")
        f.write("    }\n")
        f.write("}\n")
    
    print(f"MPC calculation functions generated: {c_filename}")

# 导出矩阵常量和计算函数
export_matrices_to_c_header()
generate_mpc_calculations_c()

print("\n=== 矩阵导出完成 ===")
print("生成的文件:")
print("1. mpc_solver_4w8d/mpc_matrices.h - 矩阵常量定义")
print("2. mpc_solver_4w8d/mpc_calculations.c - 计算函数实现")
print("这些文件包含了STM32实时计算q, l, u向量所需的所有代码")