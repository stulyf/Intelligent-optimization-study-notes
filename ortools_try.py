"""Simple Vehicles Routing Problem (VRP).

   这是一个使用OR-Tools路由库Python封装来求解车辆路径问题(VRP)的示例。
   关于VRP问题的详细描述可以在这里找到:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.

   本示例中的距离单位是米。
"""

# 导入必要的库
from ortools.constraint_solver import routing_enums_pb2  # 导入路由枚举常量
from ortools.constraint_solver import pywrapcp          # 导入约束求解器包装器


def create_data_model():
    """存储问题的数据。
    
    返回:
        data: 包含问题所有参数的字典
    """
    data = {}
    # 距离矩阵 - 表示各地点之间的距离(单位:米)
    # 矩阵中的每个元素[i][j]表示从地点i到地点j的距离
    data["distance_matrix"] = [
        # fmt: off  (这是一个格式化注释，表示下面的格式不要被自动格式化工具修改)
      [0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662],  # 从地点0到各地点的距离
      [548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210],  # 从地点1到各地点的距离
      [776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754],  # 从地点2到各地点的距离
      [696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358],  # 从地点3到各地点的距离
      [582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244],  # 从地点4到各地点的距离
      [274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708],  # 从地点5到各地点的距离
      [502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480],  # 从地点6到各地点的距离
      [194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856],  # 从地点7到各地点的距离
      [308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514],  # 从地点8到各地点的距离
      [194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468],  # 从地点9到各地点的距离
      [536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354],  # 从地点10到各地点的距离
      [502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844],  # 从地点11到各地点的距离
      [388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730],  # 从地点12到各地点的距离
      [354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536],  # 从地点13到各地点的距离
      [468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194],  # 从地点14到各地点的距离
      [776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798],  # 从地点15到各地点的距离
      [662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0],  # 从地点16到各地点的距离
        # fmt: on
    ]
    data["num_vehicles"] = 4  # 可用车辆数量
    data["depot"] = 0         # 车辆的起始位置(配送中心)索引，这里设为0表示所有车辆从第一个地点出发
    return data


def print_solution(data, manager, routing, solution):
    """在控制台打印求解结果。
    
    参数:
        data: 问题数据字典
        manager: 路由索引管理器
        routing: 路由模型
        solution: 计算得到的解决方案
    """
    # 打印目标函数值(通常是总距离)
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0  # 初始化最大路径距离
    
    # 遍历每辆车的路径
    for vehicle_id in range(data["num_vehicles"]):
        # 获取车辆的起始位置索引
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0  # 初始化当前路径距离
        
        # 遍历当前车辆的路径上的所有节点，直到到达终点
        while not routing.IsEnd(index):
            # 将路由索引转换回原始节点编号并添加到输出中
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            # 获取下一个节点的索引
            index = solution.Value(routing.NextVar(index))
            # 累加路径距离
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        
        # 添加路径终点(通常是回到配送中心)
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        print(plan_output)
        # 更新最大路径距离
        max_route_distance = max(route_distance, max_route_distance)
    
    # 打印所有路径中的最大距离
    print(f"Maximum of the route distances: {max_route_distance}m")



def main():
    """程序入口点。"""
    # 实例化数据问题
    data = create_data_model()

    # 创建路由索引管理器
    # 参数含义: 地点总数, 车辆数量, 车辆起始位置(配送中心)
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # 创建路由模型
    routing = pywrapcp.RoutingModel(manager)

    # 创建并注册距离回调函数
    def distance_callback(from_index, to_index):
        """返回两个节点之间的距离。
        
        参数:
            from_index: 起始节点的路由索引
            to_index: 目标节点的路由索引
            
        返回:
            两点间的距离
        """
        # 将路由变量索引转换为距离矩阵的节点索引
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # 查询并返回距离矩阵中对应的值
        return data["distance_matrix"][from_node][to_node]

    # 注册距离回调函数，获取回调索引用于后续设置
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # 为所有车辆设置弧(两点间连线)的成本评估器
    # 这里使用了距离作为成本
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 添加距离约束
    dimension_name = "Distance"  # 维度名称
    routing.AddDimension(
        transit_callback_index,  # 使用之前注册的回调索引
        0,    # slack变量(允许的额外距离)，这里设为0表示不允许松弛
        3000, # 车辆最大行驶距离，单位为米
        True, # 累计值从0开始(表示车辆从配送中心出发时距离为0)
        dimension_name,  # 维度名称
    )
    # 获取距离维度对象
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # 设置全局跨度成本系数
    # 这个系数用于平衡各车辆路径长度，越大则解决方案越倾向于让所有车辆的路径长度接近
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # 设置初始解启发式方法
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # 使用"最便宜弧路径"策略作为第一解方法
    # 这种策略会优先考虑成本最低的弧构建初始路径
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # 使用设定的参数求解问题
    solution = routing.SolveWithParameters(search_parameters)

    # 在控制台打印解决方案
    if solution:
        # 如果找到解决方案，则打印
        print_solution(data, manager, routing, solution)
    else:
        # 如果没找到解决方案，打印提示信息
        print("No solution found !")


if __name__ == "__main__":
    # 当脚本直接运行时，调用main函数
    main()