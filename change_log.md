## 20230914
1. 在`inspire_tactile_hand`中，因时官方给出的URDF，解析出的运动学链条（见`inspire_tactile_hand.pdf`）与建模文档不一致，包含更多关节；
2. 仿照[allegro_hand_kdl项目](https://github.com/ARQ-CRISP/allegro_hand_kdl)，创建了`inspire_hand_kdl`包；
3. 按照官方URDF结构，利用`KDL`库构建了运动学模型，主要问题是KDL的运动学算法库的传入参数需为`KDL::Chain`，即单链条结构，不包含分支，而`KDL::Tree`支持的算法不完善；
4. 按照官方URDF结构，利用`pinocchio`库构建了动力学模型，能够给出Coriolis矩阵和GeneralizedGravity向量，但没有找到计算惯量矩阵M的接口。按照`2.6.20`版本的[文档](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/index.html)，提供计算M矩阵逆的接口，但在目前安装的`2.6.17`版本中没有找到；
