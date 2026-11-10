import argparse

# 创建解析器
parser = argparse.ArgumentParser()

# 位置参数,必须输入
# nargs="?"允许不输入
parser.add_argument("number", type=int, help="输入 1 打印问候")

# 布尔开关：加 --real 是 True，不加是 False
parser.add_argument("--real", action="store_true", help="开启真机模式（仅打印）")
parser.add_argument("--try", action="store_true", help="尝试")

# 解析命令行参数
args = parser.parse_args()
print(args)  # 看看解析后的数据

if args.number == 1:
    print("hellow world")
else:
    print("你输入的不是 1")

if args.real:
    print("real 是 True:你开启了真机模式(这里只打印，不连接机器人)")
else:
    print("real 是 False:使用默认模式")
