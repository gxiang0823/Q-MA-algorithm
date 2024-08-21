import time
import numpy as np
import matplotlib.pyplot as plt
import re
import math
import gurobipy as gp
from gurobipy import GRB

class Data:
    customerNum = 0  # 任务点数量
    nodeNum = 0  # 节点数量 = 任务点数量 + 1
    PickingNum = 0  # 采摘车数目
    DeliveryNum = 0  # 运输车数目
    capacity = 0  # 采摘车容量(同质车辆，容量相同)
    corX = []  # 节点X坐标
    corY = []  # 节点X坐标
    demand = []  # 各节点待采摘量
    buffer = []  # 缓冲区容量
    serviceTime = []  # 采摘时间
    distanceMatrix = [[]]  # 节点间距离矩阵
    td_p = [[]]  # 任务点间的采摘车行驶耗时


def readData(path, customerNum):  # 从.txt文件中读取数据
    data = Data()  # 创建实例
    data.customerNum = customerNum  # 客户点数目
    if customerNum is not None:
        data.nodeNum = customerNum + 2  # 节点数目比客户数多两个
    with open(path, 'r') as f:
        lines = f.readlines()  # 按行读取数据
        count = 0
        for line in lines:
            count += 1
            if count == 5:  # 第五行为车辆数量和容量信息
                line = line[:-1]  # 去除行末的换行符并删除首尾空格
                s = re.split(r" +", line)  # 按空格分割字符串
                data.PickingNum = int(s[1])  # 读取采摘车数量
                data.DeliveryNum = int(s[2])  # 读取运输车数量
                data.capacity = float(s[3])  # 读取车辆容量
            elif count >= 10 and (customerNum is None or count <= 10 + customerNum):
                line = line[:-1]  # 去除行末的换行符并删除首尾空格
                s = re.split(r" +", line)  # 按空格分割字符串
                data.corX.append(float(s[2]))  # 节点X坐标
                data.corY.append(float(s[3]))  # 节点Y坐标
                data.demand.append(float(s[4]))  # 节点待采摘量
                data.buffer.append(float(s[5]))  # 节点缓冲区容量
                data.serviceTime.append(float(s[6]))  # 节点的采摘时间
    # 将起点的信息追加到坐标、需求、时间等列表的末尾，确保路径可以回到起点
    data.corX.append(data.corX[0])
    data.corY.append(data.corY[0])
    data.demand.append(data.demand[0])
    data.buffer.append(data.buffer[0])
    data.serviceTime.append(data.serviceTime[0])
    # 计算距离矩阵
    data.distanceMatrix = np.zeros((data.nodeNum, data.nodeNum))
    for i in range(data.nodeNum):
        for j in range(data.nodeNum):
            distance = math.sqrt((data.corX[i] - data.corX[j]) ** 2 + (data.corY[i] - data.corY[j]) ** 2)
            data.distanceMatrix[i][j] = data.distanceMatrix[j][i] = distance
    # 获取采摘车时间信息
    data.td_p = np.zeros((data.nodeNum, data.nodeNum))
    for i in range(data.nodeNum):
        for j in range(data.nodeNum):
            if i != j:
                data.td_p[i][j] = v_p * data.distanceMatrix[i][j]
    return data


def solve(data):
    # 创建模型
    model = gp.Model("FinalModel")
    # 设置 NonConvex 参数
    model.Params.NonConvex = 2

    # 变量定义
    x = model.addVars(data.nodeNum, data.nodeNum, data.PickingNum, vtype=GRB.BINARY, name="x")
    y = model.addVars(data.nodeNum, data.nodeNum, data.DeliveryNum, vtype=GRB.BINARY, name="y")
    P_A = model.addVars(data.nodeNum, vtype=GRB.CONTINUOUS, name="P_A")
    P_D = model.addVars(data.nodeNum, lb=0, vtype=GRB.CONTINUOUS, name="P_D")
    F_A = model.addVars(data.nodeNum, vtype=GRB.CONTINUOUS, name="F_A")
    F_D = model.addVars(data.nodeNum, lb=0, vtype=GRB.CONTINUOUS, name="F_D")
    F_S = model.addVars(data.nodeNum, vtype=GRB.CONTINUOUS, name="F_S")
    Q_A = model.addVars(data.nodeNum, vtype=GRB.CONTINUOUS, name="Q_A")
    Q_D = model.addVars(data.nodeNum, lb=0, vtype=GRB.CONTINUOUS, name="Q_D")
    td_f = model.addVars(data.nodeNum, data.nodeNum, vtype=GRB.CONTINUOUS, name="td_f")
    v_f_i = model.addVars(data.nodeNum, vtype=GRB.CONTINUOUS, name="v_f_i")
    fg = model.addVars(data.nodeNum, lb=0, vtype=GRB.INTEGER, name="fg")
    fw = model.addVars(data.nodeNum, vtype=GRB.BINARY, name="fw")

    # 约束条件

    # ------------------------------ 采摘车部分 ------------------------------ #

    # 每个任务点仅能被一辆采摘车服务一次
    for j in range(1, data.nodeNum - 1):
        model.addConstr(sum(x[i, j, k] for i in range(data.nodeNum) for k in range(data.PickingNum) if i != j) == 1)

    # 每个任务点仅能被一辆采摘车服务一次
    for i in range(1, data.nodeNum - 1):
        model.addConstr(sum(x[i, j, k] for j in range(data.nodeNum) for k in range(data.PickingNum) if i != j) == 1)

    # 每个任务点采摘车的出入度平衡约束
    for j in range(1, data.nodeNum - 1):
        for k in range(data.PickingNum):
            model.addConstr(sum(x[i, j, k] for i in range(data.nodeNum) if i != j) == sum(x[j, i, k] for i in range(data.nodeNum) if i != j))

    # 所有采摘车都必须发车的约束
    model.addConstr(sum(x[0, i, k] for i in range(1, data.nodeNum - 1) for k in range(data.PickingNum)) == data.PickingNum)
    model.addConstr(sum(x[j, 0, k] for j in range(1, data.nodeNum - 1) for k in range(data.PickingNum)) == data.PickingNum)

    # 任务量小于缓冲区容量
    # for i in range(1, data.nodeNum - 1):
    #     model.addConstr(data.demand[i] <= data.buffer[i])

    # 两种车型的初始状态约束
    model.addConstr(P_D[0] == 0)
    model.addConstr(F_D[0] == 30)
    model.addConstr(Q_D[0] == 0)
    model.addConstr(fg[0] == 0)

    # 采摘车到达任务点j的时刻计算方式
    for j in range(1, data.nodeNum - 1):
        model.addConstr(P_A[j] == sum(x[i, j, k] * (P_D[i] + data.td_p[i][j]) for i in range(data.nodeNum) for k in range(data.PickingNum)))

    # 采摘车离开任务点j的时刻计算方式
    for j in range(1, data.nodeNum - 1):
        model.addConstr(P_D[j] == P_A[j] + data.serviceTime[j])

    # ------------------------------ 运输车部分 ------------------------------ #

    # 每个任务点仅有一辆运输车服务，且有先后顺序
    for j in range(1, data.nodeNum - 1):
        model.addConstr(sum(y[i, j, h] for i in range(data.nodeNum) for h in range(data.DeliveryNum) if i != j) == 1)

    # 每个任务点仅有一辆运输车服务，且有先后顺序
    for i in range(1, data.nodeNum - 1):
        model.addConstr(sum(y[i, j, h] for j in range(data.nodeNum) for h in range(data.DeliveryNum) if i != j) == 1)

    # 所有运输车都必须出发，并返回depot
    model.addConstr(sum(y[0, j, h] for j in range(1, data.nodeNum - 1) for h in range(data.DeliveryNum)) == data.DeliveryNum)
    model.addConstr(sum(y[i, 0, h] for i in range(1, data.nodeNum - 1) for h in range(data.DeliveryNum)) == data.DeliveryNum)

    # 任意任务点运输车的出入度平衡约束
    for j in range(1, data.nodeNum - 1):
        for h in range(data.DeliveryNum):
            model.addConstr(sum(y[i, j, h] for i in range(data.nodeNum) if i != j) == sum(y[j, i, h] for i in range(data.nodeNum) if i != j))

    # 添加fg_j约束，表示运输车在j点超载的次数
    for j in range(1, data.nodeNum - 1):
        expr = gp.LinExpr()
        for h in range(data.DeliveryNum):
            for i in range(data.nodeNum):
                if i != j:
                    expr += y[i, j, h] * Q_D[i]
        expr += data.demand[j]
        model.addConstr(fg[j] <= expr / data.capacity)
        model.addConstr(fg[j] >= expr / data.capacity - 0.999)

    # 添加fw_j约束,用fg_j辅助判断。表示运输车在任务j点是否恰巧装满
    eps = 0.0001
    M = 1000 + eps
    for j in range(1, data.nodeNum - 1):
        expr = gp.LinExpr()
        for h in range(data.DeliveryNum):
            for i in range(data.nodeNum):
                if i != j:
                    expr += y[i, j, h] * Q_D[i]
        expr += data.demand[j]
        model.addConstr(expr >= fg[j] * data.capacity + eps - M * fw[j])
        model.addConstr(expr <= fg[j] * data.capacity + M * (1 - fw[j]))

    # 运输车在任务j开始装货的时间
    for j in range(1, data.nodeNum - 1):
        model.addConstr(F_S[j] == gp.max_(F_A[j], P_D[j]))

    # 添加F_A约束，运输车到达任务j的时刻计算方式
    for j in range(1, data.nodeNum):
        expr = gp.LinExpr()
        for h in range(data.DeliveryNum):
            for i in range(data.nodeNum):
                if i != j:
                    # 定义辅助变量
                    lin_expr_1 = model.addVar(vtype=GRB.CONTINUOUS, name=f"lin_expr_1_{i}_{j}_{h}")
                    lin_expr_2 = model.addVar(vtype=GRB.CONTINUOUS, name=f"lin_expr_2_{i}_{j}_{h}")

                    # 添加约束来表示辅助变量
                    model.addConstr(lin_expr_1 == (1 - fw[i]) * td_f[i, j])
                    model.addConstr(lin_expr_2 == fw[i] * (data.distanceMatrix[i, 0] / v_f_f + data.distanceMatrix[0, j] / v_f_e))

                    # 构建总的表达式
                    expr += y[i, j, h] * (F_D[i] + lin_expr_1 + lin_expr_2)
        model.addConstr(F_A[j] == expr)

    # 添加F_D约束，表示车辆离开任务j的时刻计算方式
    for j in range(1, data.nodeNum - 1):
        model.addConstr(F_D[j] == F_S[j] + tL * (1 - fw[j] + fg[j]) + (data.distanceMatrix[j, 0] / v_f_f + data.distanceMatrix[0, j] / v_f_e) * (fg[j] - fw[j]))

    # 运输车离开任务i的速度和ij之间的行驶时间线性关系约束
    v_f_inv_i = model.addVars(data.nodeNum, lb=0, name="v_f_inv_i")
    # 添加辅助变量f_inv_i = 1 / v_f_i
    for i in range(data.nodeNum):
        model.addConstr(v_f_i[i] * v_f_inv_i[i] == 1)
    # 添加时间约束
    for i in range(data.nodeNum):
        for j in range(data.nodeNum):
            if i != j:
                expr = gp.LinExpr()
                expr = data.distanceMatrix[i, j] * v_f_inv_i[i]
                model.addConstr(td_f[i, j] == expr)

    # 运输车的速度和载重之间的线性关系
    for i in range(data.nodeNum):
        model.addConstr(v_f_i[i] == v_f_e + (v_f_f - v_f_e) * Q_D[i] / data.capacity)

    # 运输车到达任务j时的容量
    # 假设 z[i, j, h] 是辅助变量，用于表示 y[i, j, h] * (1 - fw[i])
    a = model.addVars(data.nodeNum, data.nodeNum, data.DeliveryNum, lb=0, name="a")

    # 添加 z[i, j, h] == y[i, j, h] * (1 - fw[i]) 的约束
    for i in range(data.nodeNum):
        for j in range(data.nodeNum):
            for h in range(data.DeliveryNum):
                model.addConstr(a[i, j, h] == y[i, j, h] * (1 - fw[i]))

    # 运输车到达任务j时的容量约束
    for j in range(1, data.nodeNum - 1):
        expr = gp.LinExpr()
        for h in range(data.DeliveryNum):
            for i in range(data.nodeNum):
                if i != j:
                    expr += a[i, j, h] * Q_D[i]
        model.addConstr(Q_A[j] == expr)

    # 运输车离开任务j时的容量Q_D[j]，即完成任务j时的容量
    # aux1 和 aux2 是辅助变量，用于表示 fw[j] * data.capacity 和 (1 - fw[j]) * expr
    aux1 = model.addVars(data.nodeNum, lb=0, name="aux1")
    aux2 = model.addVars(data.nodeNum, lb=0, name="aux2")
    aux3 = model.addVars(data.nodeNum, lb=0, name="aux3")  # 新的辅助变量用于表示 (1 - fw[j]) * expr
    # 添加辅助变量的约束
    for j in range(1, data.nodeNum):
        # 辅助变量 aux1 = fw[j] * data.capacity
        model.addConstr(aux1[j] == fw[j] * data.capacity)
        expr = gp.LinExpr()
        for h in range(data.DeliveryNum):
            for i in range(data.nodeNum):
                if i != j:
                    expr += y[i, j, h] * Q_D[i]
        expr += data.demand[j]
        expr = expr - fg[j] * data.capacity
        # 辅助变量 aux2 用于表示 expr
        model.addConstr(aux2[j] == expr)
        # 辅助变量 aux3 用于表示 (1 - fw[j]) * expr
        model.addConstr(aux3[j] == (1 - fw[j]) * aux2[j])
        # 最终约束
        model.addConstr(Q_D[j] == aux1[j] + aux3[j])

    # 目标函数变量
    C_max = model.addVar(vtype=GRB.CONTINUOUS, name="C_max")

    # 辅助变量
    P_aux = model.addVars(data.nodeNum, data.PickingNum, vtype=GRB.CONTINUOUS, name="P_aux")
    V_aux = model.addVars(data.nodeNum, data.DeliveryNum, vtype=GRB.CONTINUOUS, name="V_aux")

    # 添加辅助变量约束
    for i in range(1, data.nodeNum - 1):
        for k in range(data.PickingNum):
            model.addConstr(P_aux[i, k] == P_D[i] * x[i, 0, k] + data.td_p[i, 0])

    for i in range(1, data.nodeNum - 1):
        for h in range(data.DeliveryNum):
            model.addConstr(V_aux[i, h] == F_D[i] * y[i, 0, h] + td_f[i, 0])

    # 添加目标函数约束
    for i in range(1, data.nodeNum - 1):
        for k in range(data.PickingNum):
            model.addConstr(C_max >= P_aux[i, k])

    for i in range(1, data.nodeNum - 1):
        for h in range(data.DeliveryNum):
            model.addConstr(C_max >= V_aux[i, h])

    # 目标函数
    model.setObjective(C_max, GRB.MINIMIZE)

    # 求解模型
    model.optimize()

    # 在模型不可行时调用IIS计算
    # model.computeIIS()
    # model.write("model.ilp")  # 将IIS写入文件以便检查

    # 打印IIS信息
    if model.Status == gp.GRB.INFEASIBLE:
        print('The model is infeasible; computing IIS')
        model.computeIIS()
        print('\nThe following constraint(s) cannot be satisfied:')
        for c in model.getConstrs():
            if c.IISConstr:
                print('%s' % c.constrName)

    # 输出结果
    if model.status == GRB.OPTIMAL:
        print('Optimal solution found:')
    for var in model.getVars():
        print(f'{var.varName}: {var.x}')
    else:
        print('No optimal solution found')


if __name__ == '__main__':
    # ------------------------------ 参数获取 ------------------------------ #
    tL = 2  # 平均装载时间
    Psi = 1e5  # 很大的常数
    v_p = 1  # 采摘车速度
    v_f_e = 1.2  # 运输车空载速度
    v_f_f = 0.5  # 运输车满载速度
    # ------------------------------ 数据获取 ------------------------------ #
    data_type = "C20_1"  # 算例名称
    data_path = f'./{data_type}.txt'  # 算例路径
    customerNum = 10  # 可自行设置顾客点数目，若设为None则是用完成算例进行计算
    data = readData(data_path, customerNum)  # 读取数据
    # 输出相关数据
    print('-' * 20, "Problem Information", '-' * 20)
    print(f'Data Type: {data_type}')
    print(f'Node Num: {data.nodeNum}')
    print(f'Customer Num: {data.customerNum}')
    print(f'Picking Vehicle Num: {data.PickingNum}')
    print(f'Delivery Truck Num: {data.DeliveryNum}')
    print(f'Vehicle Capacity: {data.capacity}')
    print('-' * 61)
    # 建模求解
    solve(data)
