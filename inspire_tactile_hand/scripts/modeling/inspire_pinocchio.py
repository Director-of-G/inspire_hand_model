import numpy as np
import pinocchio as pin
from scipy import stats
import os

import os
URDF_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "urdf"
)

# 符号变量
DISABLE_HAND_GRAVITY = True

# 关节限位数据
ARM_UPPER_LIMITS = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
ARM_LOWER_LIMITS = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
FINGER_UPPER_LIMITS = [0.0089, 0.00875, 0.0083, 0.0079, 0.006, 0.0042]
FINGER_LOWER_LIMITS = [-0.001, -0.001, -0.0015, -0.002, -0.0027, -0.0058]
LOWER_LIMITS = np.r_[ARM_LOWER_LIMITS, FINGER_LOWER_LIMITS]
UPPER_LIMITS = np.r_[ARM_UPPER_LIMITS, FINGER_UPPER_LIMITS]


# 生成测试数据：手指关节角度、角速度和角加速度
def generate_test_data():
    """
        Generate test data for the arm-hand system
    """
    q_rand = np.random.uniform(LOWER_LIMITS, UPPER_LIMITS)
    dq_rand = np.random.normal(
        loc=np.zeros(12,),
        scale=[0.1]*6 + [0.001]*6
    )
    ddq_rand = np.random.normal(
        loc=np.zeros(12,),
        scale=[1e-3]*6 + [1e-5]*6
    )

    return q_rand, dq_rand, ddq_rand


# 求解指定手指的被动关节与主动关节间速度的比例关系
def get_k_passive(finger='0', index_active=0):
    """
        Get the ratio: dqa/dq for specific finger
        Use linear regression to fit the data
        finger: str, "0", "4-0", "4-1", "6", "7", "8"
        index_active: int, index of active joint, in the finger's jpos
    """
    data = np.load("../data/finger{}.npy".format(finger))

    data_active = data[:, index_active]
    data_passive = np.delete(data, index_active, axis=1)

    k_passive = np.zeros((data_passive.shape[1],))
    for i in range(data_passive.shape[1]):
        result = stats.linregress(data_active, data_passive[:, i])
        k_passive[i] = result.slope

    # k_all = np.insert(k_passive, index_active, 1.0)

    return k_passive


# 求解指定手指的被动关节角度、角速度和角加速度
def get_jpos_passive(q=0.0, dq=0.0, ddq=0.0, finger='0', index_active=0):
    data = np.load("../data/finger{}.npy".format(finger))

    data_active = data[:, index_active]

    q_all = np.zeros(data.shape[1])
    dq_all = np.zeros(data.shape[1])
    ddq_all = np.zeros(data.shape[1])

    # 填入主动关节的角度、角速度和角加速度
    q_all[index_active] = q
    dq_all[index_active] = dq
    ddq_all[index_active] = ddq

    for i in range(data.shape[1]):
        if i == index_active:
            continue
        result = stats.linregress(data_active, data[:, i])
        slope_ = result.slope
        intercept_ = result.intercept
        q_all[i] = slope_ * q + intercept_
        dq_all[i] = slope_ * dq
        ddq_all[i] = slope_ * ddq

    return np.vstack((q_all, dq_all, ddq_all))


# 创建主被动投影关系矩阵
def make_passive_projection_matrix():
    """
        Return: matrix \Gamma defined in Eq. (42)
    """
    nq = 36
    nqa = 12
    index_nqa = [0, 1, 2, 3, 4, 5, 7, 12, 16, 22, 27, 32]

    Gamma = np.zeros((nq, nqa))
    Gamma[index_nqa, range(nqa)] = 1                                # actuated joints
    Gamma[[6,8,9,10], 6] = get_k_passive(finger="0", index_active=1)
    Gamma[[11,13], 7] = get_k_passive(finger="4-0", index_active=1)
    Gamma[[14,15,17,18,19,20], 8] = get_k_passive(finger="4-1", index_active=2)
    Gamma[[21,23,24,25], 9] = get_k_passive(finger="6", index_active=1)
    Gamma[[26,28,29,30], 10] = get_k_passive(finger="7", index_active=1)
    Gamma[[31,33,34,35], 11] = get_k_passive(finger="8", index_active=1)

    return Gamma


# 从主动关节角度、角速度和角加速度计算被动关节角度、角速度和角加速度
def compute_all_joints_from_active_joints(q, dq, ddq):
    nq = 36
    nqa = 12

    assert len(q) == nqa and len(dq) == nqa and len(ddq) == nqa

    q_all = np.zeros(nq,)
    dq_all = np.zeros(nq,)
    ddq_all = np.zeros(nq,)

    # 填入机械臂的6个自由度
    q_all[:6] = q[:6]
    dq_all[:6] = dq[:6]
    ddq_all[:6] = ddq[:6]

    # 计算灵巧手的自由度
    # Finger0
    joint_data_ = get_jpos_passive(q[6+0], dq[6+0], ddq[6+0], finger="0", index_active=1)
    q_all[6:11] = joint_data_[0]
    dq_all[6:11] = joint_data_[1]
    ddq_all[6:11] = joint_data_[2]

    # Finger4-0
    joint_data_ = get_jpos_passive(q[6+1], dq[6+1], ddq[6+1], finger="4-0", index_active=1)
    q_all[11:14] = joint_data_[0]
    dq_all[11:14] = joint_data_[1]
    ddq_all[11:14] = joint_data_[2]

    # Finger4-1
    joint_data_ = get_jpos_passive(q[6+2], dq[6+2], ddq[6+2], finger="4-1", index_active=2)
    q_all[14:21] = joint_data_[0]
    dq_all[14:21] = joint_data_[1]
    ddq_all[14:21] = joint_data_[2]

    # Finger6
    joint_data_ = get_jpos_passive(q[6+3], dq[6+3], ddq[6+3], finger="6", index_active=1)
    q_all[21:26] = joint_data_[0]
    dq_all[21:26] = joint_data_[1]
    ddq_all[21:26] = joint_data_[2]

    # Finger7
    joint_data_ = get_jpos_passive(q[6+4], dq[6+4], ddq[6+4], finger="7", index_active=1)
    q_all[26:31] = joint_data_[0]
    dq_all[26:31] = joint_data_[1]
    ddq_all[26:31] = joint_data_[2]

    # Finger8
    joint_data_ = get_jpos_passive(q[6+5], dq[6+5], ddq[6+5], finger="8", index_active=1)
    q_all[31:36] = joint_data_[0]
    dq_all[31:36] = joint_data_[1]
    ddq_all[31:36] = joint_data_[2]

    return np.vstack((q_all, dq_all, ddq_all))


## Model test API
def test_arm_hand_model(
    model,
    q, dq=None, ddq=None,
    tau_gt=None,
    disable_hand_grav = True
):
    """
        q: generalized jpos
        dq: generalized jvel
        ddq: generalized jacc
    """
    q, dq, ddq = compute_all_joints_from_active_joints(q, dq, ddq)
    tau = pin.rnea(model, data, q, dq, ddq)
    grav = pin.computeGeneralizedGravity(model, data, q)
    if disable_hand_grav:
        grav[:6] = 0.0
        tau = tau - grav
    tau_active = np.matmul(Gamma.T, tau)

    if tau_gt is None:
        tau_residual = np.zeros_like(tau_active)
    else:
        tau_residual = tau_gt - tau_active

    print("tau (model output): ", tau_active)
    print("tau (residual): ", tau_residual)

    return tau_residual


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
# import pdb; pdb.set_trace()

nq = 36
nqa = 12
index_nqa = [0, 1, 2, 3, 4, 5, 7, 12, 16, 22, 27, 32]

Gamma = make_passive_projection_matrix()


## 计算逆动力学问题
# 随机生成主动关节角度、角速度和角加速度
q_rand, dq_rand, ddq_rand = generate_test_data()

# 测试模型精度
tau_r = test_arm_hand_model(
    model,
    q_rand, dq_rand, ddq_rand,
    disable_hand_grav = DISABLE_HAND_GRAVITY
)

# # 计算全部关节角度、角速度和角加速度
# q_rand, dq_rand, ddq_rand = compute_all_joints_from_active_joints(q_rand, dq_rand, ddq_rand)

# # 计算全部关节的广义力、以及重力贡献的部分
# tau = pin.rnea(model, data, q_rand, dq_rand, ddq_rand)
# grav = pin.computeGeneralizedGravity(model, data, q_rand)
# if DISABLE_HAND_GRAVITY:
#     grav[:6] = 0.0
#     tau = tau - grav

# # 计算主动关节的广义力
# tau_active = np.matmul(Gamma.T, tau)

