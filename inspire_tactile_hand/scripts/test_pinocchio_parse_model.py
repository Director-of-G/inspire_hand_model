import pinocchio as pin
import os

import os
URDF_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "urdf")

# 指定 URDF 文件路径
urdf_filename = os.path.join(URDF_DIR, "ur5_inspire_hand.urdf")
print(urdf_filename)

# 从 URDF 中加载模型
model = pin.buildModelFromUrdf(urdf_filename)

# 创建数据对象，用于后续的算法计算
data = model.createData()

print("模型加载成功！")
import pdb; pdb.set_trace()
