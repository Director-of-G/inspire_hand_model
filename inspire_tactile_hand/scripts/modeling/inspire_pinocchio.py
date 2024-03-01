import numpy as np
import pinocchio as pin
import os

import os
URDF_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "urdf"
)

# 指定 URDF 文件路径
urdf_filename = os.path.join(URDF_DIR, "ur5_inspire_hand.urdf")
print(urdf_filename)

# 从 URDF 中加载模型
model = pin.buildModelFromUrdf(urdf_filename)

# 创建数据对象，用于后续的算法计算
data = model.createData()

## 模型定义
# ------------------------------
# >>> model joints (njoints=37)
# BASE: universe
# UR5: shoulder_pan_joint(0), shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
# FINGER0: joint001, joint012(7), joint023, joint034, joint005
# FINGER4: joint401, joint412(12), joint423 || joint434, joint445, joint456(16), joint467, joint478, joint449, joint4350
# FINGER6: joint601, joint612(22), joint623, joint634, joint605
# FINGER7: joint701, joint712(27), joint723, joint734, joint705
# FINGER8: joint801, joint812(32), joint823, joint834, joint805
# >>> actuated joints (njoints=12)
# shoulder_pan_joint shoulder_lift_joint elbow_joint wrist_1_joint wrist_2_joint wrist_3_joint
# FINGER0: joint012
# FINGER4: joint412 joint456
# FINGER6: joint612
# FINGER7: joint712
# FINGER8: joint812
# ------------------------------

print("模型加载成功！")
for i in range(model.njoints):
    print(model.names[i])
import pdb; pdb.set_trace()

nq = 36
nqa = 12
index_nqa = [0, 1, 2, 3, 4, 5, 7, 12, 16, 22, 27, 32]

Gamma = np.zeros((nq, nqa))
Gamma[index_nqa, range(nqa)] = 1                                # actuated joints
Gamma[[6,8,9,10], 7] = [1.0, 1.0, 1.0, 1.0]                     # FINGER0
Gamma[[11,13], 12] = [1.0, 1.0]                                 # FINGER4-1
Gamma[[14,15,17,18,19,20], 16] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] # FINGER4-2
Gamma[[21,23,24,25], 22] = [1.0, 1.0, 1.0, 1.0]                 # FINGER6
Gamma[[26,28,29,30], 27] = [1.0, 1.0, 1.0, 1.0]                 # FINGER7
Gamma[[31,33,34,35], 32] = [1.0, 1.0, 1.0, 1.0]                 # FINGER8
