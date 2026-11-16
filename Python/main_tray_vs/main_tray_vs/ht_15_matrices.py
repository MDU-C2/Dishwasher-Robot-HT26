"""15 张未补偿 HT，可直接导入。

p_B = R @ p_C + t，单位 mm。
相机点是 ArUco 中心，机器人点是 RAPID robtarget.trans。
没有加入法兰到标记的 [0, 35, 60] mm。

最好：HT_NEW_COMBINED（第 14 张）。
单轮最好：HT_NEW_RUN3_MEDIAN（第 12 张）。
最差：HT_OLD_RUN1_MEAN（第 1 张）。旋转偏离最大：HT_OLD_RUN3_MEAN（第 5 张）。
固定误差：HT_OLD_COMBINED（第 13 张），预测 New 时整组约 35 mm。
第 4、6 张的中位数误差也停在约 30–35 mm。New 六张单轮的误差地板约 10 mm。
"""

import numpy as np


def build_homogeneous(rotation_matrix, translation_vector):
    """把 3x3 旋转和 3x1 平移合成 4x4。平移单位 mm。"""
    homogeneous = np.eye(4, dtype=np.float64)
    homogeneous[:3, :3] = np.asarray(rotation_matrix, dtype=np.float64)
    homogeneous[:3, 3] = np.asarray(translation_vector, dtype=np.float64).reshape(3)
    return homogeneous

# 1/15  HT_Old_Run1_Mean
# Old 第 1 轮，每个点用窗口内有效帧 XYZ 的平均数。53 个点，含 17 个 WARNING。
# 最差。5 折验证 RMSE 1391 mm，最大误差 9812 mm，相对 Global 旋转差 114 deg。
# 少数成功批次里的野深度把平均数拉飞。不要使用。
rotation_matrix = np.array([
    [0.01831478504609732, -0.7074425184357911, -0.7065335460952588],
    [-0.8221249995871275, 0.3915056440476977, -0.4133204758207166],
    [0.569012349341015, 0.588428766961663, -0.5744358384618733],
], dtype=np.float64)
translation_vector = np.array(
    [855.6738864046092, 339.4844299133803, 465.7559749098718],
    dtype=np.float64,
)
HT_OLD_RUN1_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 2/15  HT_Old_Run1_Median
# 同一轮改用帧中位数。验证 RMSE 159 mm，验证中位数 34 mm，最大误差 1083 mm。
# 比第 1 张好一个数量级，但仍是三轮 Old Median 里最松的一张（相对 Global 旋转差 9.7 deg，平移差 91 mm）。
rotation_matrix = np.array([
    [0.06852747541109822, -0.5318194461810458, 0.8440806014697094],
    [-0.9951213231841887, 0.02375424177455546, 0.09575640000472022],
    [-0.07097561030332504, -0.8465245493554225, -0.5275970527596501],
], dtype=np.float64)
translation_vector = np.array(
    [-66.14401567185149, -47.43149307109331, 402.8490322518624],
    dtype=np.float64,
)
HT_OLD_RUN1_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 3/15  HT_Old_Run2_Mean
# Old 第 2 轮，帧平均数。
# 与第 1 张同级，不可用。验证 RMSE 1300 mm，最大误差 9183 mm，相对 Global 旋转差 105 deg。
rotation_matrix = np.array([
    [-0.05756181797477241, -0.8088023828953823, -0.5852566467236323],
    [-0.7887538869386013, 0.3962159621278796, -0.4699789539908433],
    [0.612008123271814, 0.4345706119583219, -0.66075293436464],
], dtype=np.float64)
translation_vector = np.array(
    [771.0841493197508, 365.942386551712, 515.4680532753441],
    dtype=np.float64,
)
HT_OLD_RUN2_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 4/15  HT_Old_Run2_Median
# Old 第 2 轮，帧中位数。验证中位数 32 mm，验证 RMSE 92 mm，最大误差 414 mm。
# 固定误差约 30 mm。中位数停在 32 mm，和 Old 第 3 轮 Median、以及 Old 合并矩阵
# 预测 New 时的约 35 mm 是同一量级。不是最好，但这层误差是稳定的，不是随机炸开。
rotation_matrix = np.array([
    [-0.0006205212096901543, -0.4900577775254819, 0.8716897324393663],
    [-0.9999793197760416, 0.005883921134828845, 0.002596053220638675],
    [-0.006401169711357216, -0.8716700947943921, -0.4900512941187503],
], dtype=np.float64)
translation_vector = np.array(
    [-75.66104492078932, 6.637931281643061, 379.8864369424275],
    dtype=np.float64,
)
HT_OLD_RUN2_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 5/15  HT_Old_Run3_Mean
# Old 第 3 轮，帧平均数。
# 最差之一，而且是 15 张里旋转偏离最大的：相对 Global 169.5 deg，平移差 1168 mm。
# 验证 RMSE 1065 mm。不要使用。
rotation_matrix = np.array([
    [0.01273544212206357, 0.2413560283170001, -0.9703530677588439],
    [-0.9998299113933476, -0.009872901848784886, -0.01557800026495237],
    [-0.01334004486972014, 0.9703864144783394, 0.2411892406363647],
], dtype=np.float64)
translation_vector = np.array(
    [1019.142775055079, 51.11972802666973, -39.56340334851546],
    dtype=np.float64,
)
HT_OLD_RUN3_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 6/15  HT_Old_Run3_Median
# Old 第 3 轮，帧中位数。验证中位数 35 mm，验证 RMSE 101 mm，最大误差 417 mm。
# 固定误差约 30–35 mm，与第 4 张、第 13 张同一水平。最大误差仍会被 s48 一类点拉高。
rotation_matrix = np.array([
    [0.0006281331735384584, -0.4817874909920819, 0.8762878630748404],
    [-0.9999811268189092, 0.005052887575323413, 0.003494901019284786],
    [-0.006111583649084523, -0.8762735199985821, -0.4817752242432092],
], dtype=np.float64)
translation_vector = np.array(
    [-81.66163608444737, 5.580272708459737, 376.5135519947153],
    dtype=np.float64,
)
HT_OLD_RUN3_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 7/15  HT_New_Run1_Mean
# New 第 1 轮，帧平均数。验证 RMSE 16.3 mm，验证中位数 9.9 mm，最大误差 67 mm。
# 不是总排名第一。误差稳定在约 10 mm，New 三轮彼此旋转差小于 1 deg。
rotation_matrix = np.array([
    [0.004050474610010031, -0.395210887150983, 0.9185814870400812],
    [-0.9999752900301533, -0.00687875190348592, 0.001449862531987259],
    [0.00574569269497015, -0.9185646615506081, -0.3952289837118119],
], dtype=np.float64)
translation_vector = np.array(
    [-80.68928353402373, 8.096977604701195, 322.5271910551188],
    dtype=np.float64,
)
HT_NEW_RUN1_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 8/15  HT_New_Run1_Median
# New 第 1 轮，帧中位数。验证 RMSE 17.7 mm，验证中位数 9.7 mm。
# 和同轮 Mean 几乎一样，因为这轮大多数窗口本身很稳。误差地板约 10 mm。
rotation_matrix = np.array([
    [0.009327009202570978, -0.3931973418446285, 0.9194067963995338],
    [-0.9999551614672457, -0.005173323322659086, 0.007931694700048651],
    [0.001637667350442645, -0.9194395505372375, -0.3932279630869757],
], dtype=np.float64)
translation_vector = np.array(
    [-80.13709099126663, 4.625590098452065, 320.88348177353],
    dtype=np.float64,
)
HT_NEW_RUN1_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 9/15  HT_New_Run2_Mean
# New 第 2 轮，帧平均数。验证 RMSE 20.5 mm，验证中位数 10.6 mm，最大误差 97 mm。
# 仍在 New 约 10 mm 的误差带里，比第 7 张略松。
rotation_matrix = np.array([
    [-0.002597731108439694, -0.3876317471376878, 0.9218106532276962],
    [-0.9999744383587065, -0.005133658742591244, -0.004976763717912172],
    [0.00666141293402215, -0.9218000185283671, -0.3876085027689978],
], dtype=np.float64)
translation_vector = np.array(
    [-84.04769438994447, 10.90679989710399, 319.0687334399627],
    dtype=np.float64,
)
HT_NEW_RUN2_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 10/15  HT_New_Run2_Median
# New 第 2 轮，帧中位数。验证 RMSE 21.3 mm，验证中位数 10.8 mm，最大误差 121 mm。
# 中位数仍约 11 mm。单轮最大误差是 New 六张里偏高的，主要是个别点，不是整组偏移。
rotation_matrix = np.array([
    [-0.0005869010895230238, -0.3886063414818832, 0.9214036937776934],
    [-0.9999860674062694, -0.004605617204006339, -0.002579395959049607],
    [0.005246002330777363, -0.9213923700846649, -0.3885982241458513],
], dtype=np.float64)
translation_vector = np.array(
    [-82.83560926848418, 9.611510893627841, 319.0308908904267],
    dtype=np.float64,
)
HT_NEW_RUN2_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 11/15  HT_New_Run3_Mean
# New 第 3 轮，帧平均数。验证 RMSE 16.2 mm，验证中位数 10.1 mm，最大误差 52 mm。
# 六张 Mean 里最大误差最小。误差水平与第 12 张、第 14 张同一族，约 10 mm。
rotation_matrix = np.array([
    [0.00361799427859304, -0.3948560259703314, 0.9187358863527194],
    [-0.9999887469951038, -0.004247813592985132, 0.002112335873371531],
    [0.003068550237938367, -0.9187331902323959, -0.3948669512188138],
], dtype=np.float64)
translation_vector = np.array(
    [-80.94058343645128, 7.330829145396862, 322.113730934712],
    dtype=np.float64,
)
HT_NEW_RUN3_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 12/15  HT_New_Run3_Median
# New 第 3 轮，帧中位数。
# 单轮最好。12 张单轮里验证 RMSE 最低，15.5 mm；验证中位数 10.1 mm，最大误差 50 mm。
# 和推荐的第 14 张属于同一族，只是没用上另外两轮。
rotation_matrix = np.array([
    [0.005650500498997963, -0.3973578849441982, 0.9176463279045931],
    [-0.9999820525749135, -0.004072922431638321, 0.004393840111901713],
    [0.001991575299584984, -0.9176546859116119, -0.3973737674435579],
], dtype=np.float64)
translation_vector = np.array(
    [-79.34284113735981, 6.146410541936348, 322.8155589118912],
    dtype=np.float64,
)
HT_NEW_RUN3_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 13/15  HT_Old_Combined
# Old 三轮合成：每个物理点先取该轮帧中位数，三轮再取中位数。56 个点。
# 组内验证中位数 36 mm，验证 RMSE 100 mm。
# 固定误差。用它预测 New 的 61 个点时，RMSE 36.8 mm、中位数 34.9 mm；
# 去掉 New 的 WARNING 后仍是约 35 mm。误差铺在整组点上，不是一两个坏点。
# 表现不是最好，但这 35 mm 是稳定的系统偏差。
rotation_matrix = np.array([
    [0.003664500737471044, -0.4857617321186664, 0.8740835835567545],
    [-0.9999582304209516, 0.00553886606350392, 0.007270376615402938],
    [-0.008373102635481583, -0.8740737157538854, -0.4857211448768236],
], dtype=np.float64)
translation_vector = np.array(
    [-79.79064633712261, 3.734949819578572, 378.9227052744367],
    dtype=np.float64,
)
HT_OLD_COMBINED = build_homogeneous(rotation_matrix, translation_vector)

# 14/15  HT_New_Combined
# New 三轮合成，规则与第 13 张相同。61 个点都有观测。
# 组内验证 RMSE 17.6 mm，验证中位数 9.9 mm。
# 预测清洗后的 Old 点，中位数误差 11.5 mm，RMSE 14.3 mm。
# 最好，推荐作为后续实验的候选。
# 仍是未补偿实验矩阵：相机点是 ArUco 中心，机器人点是 robtarget.trans，没有加 [0, 35, 60] mm。
rotation_matrix = np.array([
    [0.002487476946309068, -0.391837519315182, 0.9200310706249906],
    [-0.9999923300939482, -0.003758037845548234, 0.001103134093083655],
    [0.003025262255983282, -0.9200267580937393, -0.3918438619907255],
], dtype=np.float64)
translation_vector = np.array(
    [-81.46809799545474, 7.756143525855466, 320.382955603828],
    dtype=np.float64,
)
HT_NEW_COMBINED = build_homogeneous(rotation_matrix, translation_vector)

# 15/15  HT_Global
# Old 与 New 的物理点合在一起，117 个点，代表值仍是中位数。Old 的 s15 与 New 的 s15 没有合并。
# 验证 RMSE 67 mm，验证中位数 17.5 mm。
# 不是最好。它覆盖的姿态最多，但 Old 的可疑点把验证中位数从 New 单独的约 10 mm 拉到 17 mm。
# 不要因为它用了六次采样就当成最准。
rotation_matrix = np.array([
    [-0.001228594413533647, -0.4133322585248082, 0.9105794499210647],
    [-0.9999741099002089, -0.005948340192757313, -0.004049293547935558],
    [0.007090139988262434, -0.9105608498676696, -0.4133142492138572],
], dtype=np.float64)
translation_vector = np.array(
    [-86.38441223878368, 9.999333157917274, 335.8371091319808],
    dtype=np.float64,
)
HT_GLOBAL = build_homogeneous(rotation_matrix, translation_vector)

MATRICES = {
    "HT_Old_Run1_Mean": HT_OLD_RUN1_MEAN,
    "HT_Old_Run1_Median": HT_OLD_RUN1_MEDIAN,
    "HT_Old_Run2_Mean": HT_OLD_RUN2_MEAN,
    "HT_Old_Run2_Median": HT_OLD_RUN2_MEDIAN,
    "HT_Old_Run3_Mean": HT_OLD_RUN3_MEAN,
    "HT_Old_Run3_Median": HT_OLD_RUN3_MEDIAN,
    "HT_New_Run1_Mean": HT_NEW_RUN1_MEAN,
    "HT_New_Run1_Median": HT_NEW_RUN1_MEDIAN,
    "HT_New_Run2_Mean": HT_NEW_RUN2_MEAN,
    "HT_New_Run2_Median": HT_NEW_RUN2_MEDIAN,
    "HT_New_Run3_Mean": HT_NEW_RUN3_MEAN,
    "HT_New_Run3_Median": HT_NEW_RUN3_MEDIAN,
    "HT_Old_Combined": HT_OLD_COMBINED,
    "HT_New_Combined": HT_NEW_COMBINED,
    "HT_Global": HT_GLOBAL,
}

BEST = "HT_New_Combined"
BEST_SINGLE_RUN = "HT_New_Run3_Median"
WORST = "HT_Old_Run1_Mean"
LARGEST_ROTATION_OUTLIER = "HT_Old_Run3_Mean"
FIXED_BIAS = "HT_Old_Combined"
