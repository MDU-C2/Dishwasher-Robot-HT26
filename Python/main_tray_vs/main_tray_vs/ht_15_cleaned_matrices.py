"""15 张去掉 WARNING 的 HT，对应 matrices/sensitivity/。可直接导入。

p_B = R @ p_C + t，单位 mm。
相机点是 ArUco 中心，机器人点是 RAPID robtarget.trans。
没有加入法兰到标记的 [0, 35, 60] mm。
失败批次和 WARNING 都不在这 15 张里。带 WARNING 的另外 15 张在 ht_15_matrices.py。

最好：HT_NEW_COMBINED_CLEANED（第 14 张）。
单轮最好：HT_NEW_RUN2_MEAN_CLEANED（第 9 张），验证 RMSE 11.1 mm。
这组里最松：HT_OLD_RUN1_MEDIAN_CLEANED（第 2 张），验证 RMSE 13.6 mm。
15 张彼此旋转差最大 1.24 deg（第 10 张与第 13 张）。
固定误差：HT_OLD_COMBINED_CLEANED（第 13 张），预测清洗后的 New 时整组约 11 mm。
"""

import numpy as np


def build_homogeneous(rotation_matrix, translation_vector):
    """把 3x3 旋转和 3x1 平移合成 4x4。平移单位 mm。"""
    homogeneous = np.eye(4, dtype=np.float64)
    homogeneous[:3, :3] = np.asarray(rotation_matrix, dtype=np.float64)
    homogeneous[:3, 3] = np.asarray(translation_vector, dtype=np.float64).reshape(3)
    return homogeneous

# 1/15  HT_Old_Run1_Mean_CleanedSensitivity
# Old 第 1 轮，每个点用窗口内有效帧 XYZ 的平均数。
# 去掉 WARNING 和失败批次后剩 40 个点，排除 21 条观测。
# 5 折验证 RMSE 13.3 mm，验证中位数 11.8 mm，最大误差 40.7 mm。
# 相对清洗后的 Global 旋转差 0.22 deg。s48 的 9.2 m 平均数不在这张里。
rotation_matrix = np.array([
    [0.03144888808725191, -0.4024956449966385, 0.9148815350616795],
    [-0.9994496933209274, -0.003003108817537677, 0.033034706872230084],
    [-0.010548836844867071, -0.915416974441795, -0.4023685933880158],
], dtype=np.float64)
translation_vector = np.array(
    [-78.86380668989926, -11.236187333358469, 324.1679655267853],
    dtype=np.float64,
)
HT_OLD_RUN1_MEAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 2/15  HT_Old_Run1_Median_CleanedSensitivity
# 同一轮改用帧中位数。40 个点，排除 21 条观测。
# 这 15 张里验证 RMSE 最高：13.6 mm。验证中位数 11.6 mm，最大误差 40.5 mm。
# 去掉野深度之后，它和其余 Old 单轮同级，大约 14 mm。
rotation_matrix = np.array([
    [0.032893103162021875, -0.4022028098919006, 0.9149595310610367],
    [-0.9993958989890228, -0.0029593703290260983, 0.034627723159035034],
    [-0.011219663466241082, -0.9155458163534556, -0.4020571816413542],
], dtype=np.float64)
translation_vector = np.array(
    [-79.1071096900817, -12.143163246922025, 324.0853728958707],
    dtype=np.float64,
)
HT_OLD_RUN1_MEDIAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 3/15  HT_Old_Run2_Mean_CleanedSensitivity
# Old 第 2 轮，帧平均数。41 个点，排除 20 条观测。
# 验证 RMSE 13.2 mm，验证中位数 10.8 mm，最大误差 41.9 mm。
# 相对清洗后的 Global 旋转差 0.36 deg。
rotation_matrix = np.array([
    [0.03359344576813546, -0.4013509697179099, 0.9153080790137913],
    [-0.9993841809104165, -0.004201599547923804, 0.03483684123533278],
    [-0.010136042000714211, -0.9159147043626565, -0.40124495633568713],
], dtype=np.float64)
translation_vector = np.array(
    [-78.38665099145993, -11.438209731765419, 323.2237234179735],
    dtype=np.float64,
)
HT_OLD_RUN2_MEAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 4/15  HT_Old_Run2_Median_CleanedSensitivity
# Old 第 2 轮，帧中位数。41 个点，排除 20 条观测。
# 验证 RMSE 13.3 mm，验证中位数 10.7 mm，最大误差 42.3 mm。
# 最大误差是这 15 张里最高的一个，来自单个点；整组中位数约 11 mm。
rotation_matrix = np.array([
    [0.03330126776592841, -0.40226614921881676, 0.9149169201396627],
    [-0.9993908009246085, -0.003837750681956591, 0.034688596065158055],
    [-0.010442824926738802, -0.9155147278238445, -0.4021488910157303],
], dtype=np.float64)
translation_vector = np.array(
    [-78.029018869955, -11.343350804478526, 323.655756614445],
    dtype=np.float64,
)
HT_OLD_RUN2_MEDIAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 5/15  HT_Old_Run3_Mean_CleanedSensitivity
# Old 第 3 轮，帧平均数。39 个点，排除 22 条观测。
# 验证 RMSE 13.2 mm，验证中位数 11.2 mm，最大误差 39.4 mm。
# 相对清洗后的 Global 旋转差 0.30 deg。15 张彼此最大旋转差 1.24 deg。
rotation_matrix = np.array([
    [0.033014521463702554, -0.40149281396036424, 0.9152669346757322],
    [-0.9993782263233291, -0.00192024211312909, 0.03520615601091265],
    [-0.012377484532829726, -0.9158601601818999, -0.40130657217173044],
], dtype=np.float64)
translation_vector = np.array(
    [-77.71181611509809, -11.442509656657151, 322.99756400690706],
    dtype=np.float64,
)
HT_OLD_RUN3_MEAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 6/15  HT_Old_Run3_Median_CleanedSensitivity
# Old 第 3 轮，帧中位数。39 个点，排除 22 条观测。
# 验证 RMSE 13.6 mm，验证中位数 11.2 mm，最大误差 39.4 mm。
# 和同轮平均数同级。
rotation_matrix = np.array([
    [0.032617371770603254, -0.4001435059137653, 0.915871869714219],
    [-0.999399636262694, -0.002346880484882209, 0.03456673530991072],
    [-0.011682212837215725, -0.9164494895121228, -0.39997982333637244],
], dtype=np.float64)
translation_vector = np.array(
    [-77.70027994270129, -11.096335030861958, 322.1345591024467],
    dtype=np.float64,
)
HT_OLD_RUN3_MEDIAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 7/15  HT_New_Run1_Mean_CleanedSensitivity
# New 第 1 轮，帧平均数。45 个点，排除 16 条观测。
# 验证 RMSE 12.0 mm，验证中位数 8.9 mm，最大误差 38.0 mm。
# 误差停在约 9–12 mm，和其余 New 单轮同一族。
rotation_matrix = np.array([
    [0.028373932166498684, -0.4108660077337764, 0.9112541048809162],
    [-0.9995970304314723, -0.012423813465061192, 0.025523040797843163],
    [0.0008347011404758573, -0.9116110862357334, -0.4110529536764242],
], dtype=np.float64)
translation_vector = np.array(
    [-72.96495940482299, -1.750830190223823, 328.24141100324175],
    dtype=np.float64,
)
HT_NEW_RUN1_MEAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 8/15  HT_New_Run1_Median_CleanedSensitivity
# New 第 1 轮，帧中位数。45 个点，排除 16 条观测。
# 验证 RMSE 12.1 mm，验证中位数 9.1 mm，最大误差 38.3 mm。
# 和同轮平均数几乎一样。
rotation_matrix = np.array([
    [0.028715820141310695, -0.4110088599924642, 0.9111789718169023],
    [-0.9995874617686297, -0.012313382708134047, 0.02594777218244095],
    [0.0005549311315365368, -0.9115481872144655, -0.4111928920074633],
], dtype=np.float64)
translation_vector = np.array(
    [-72.8106786287023, -1.9626382102156583, 328.23226868445795],
    dtype=np.float64,
)
HT_NEW_RUN1_MEDIAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 9/15  HT_New_Run2_Mean_CleanedSensitivity
# New 第 2 轮，帧平均数。47 个点，排除 14 条观测。
# 单轮最好。15 张里验证 RMSE 最低，11.1 mm；验证中位数 8.9 mm，最大误差 36.8 mm。
# 只用了这一轮，没有用上另外两轮。
rotation_matrix = np.array([
    [0.031973486753756275, -0.4081142101175399, 0.9123708060021128],
    [-0.9994685437021028, -0.018855002178875653, 0.02659171003965988],
    [0.006350298796601704, -0.9127361504799574, -0.4085001754127199],
], dtype=np.float64)
translation_vector = np.array(
    [-73.89249845543327, -2.035982816083525, 327.61021107917924],
    dtype=np.float64,
)
HT_NEW_RUN2_MEAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 10/15  HT_New_Run2_Median_CleanedSensitivity
# New 第 2 轮，帧中位数。47 个点，排除 14 条观测。
# 验证 RMSE 11.3 mm，验证中位数 9.3 mm，最大误差 38.2 mm。
# 和同轮平均数同级。
rotation_matrix = np.array([
    [0.030696269989302912, -0.40821807031875923, 0.9123682075094315],
    [-0.9995035733744779, -0.019016159015000814, 0.02511956424715784],
    [0.007095478870007499, -0.9126863605650878, -0.4085991451509176],
], dtype=np.float64)
translation_vector = np.array(
    [-73.97364474485641, -1.2901073297233268, 327.72316811482443],
    dtype=np.float64,
)
HT_NEW_RUN2_MEDIAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 11/15  HT_New_Run3_Mean_CleanedSensitivity
# New 第 3 轮，帧平均数。47 个点，排除 14 条观测。
# 验证 RMSE 11.6 mm，验证中位数 9.0 mm，最大误差 38.5 mm。
rotation_matrix = np.array([
    [0.029590821270854398, -0.409788433062377, 0.9117005119143008],
    [-0.9995620538665035, -0.011867188159155155, 0.027108491573183093],
    [-0.0002894247647741972, -0.912103398729265, -0.409960127646371],
], dtype=np.float64)
translation_vector = np.array(
    [-73.5933493280366, -3.054265065437761, 328.06380026030325],
    dtype=np.float64,
)
HT_NEW_RUN3_MEAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 12/15  HT_New_Run3_Median_CleanedSensitivity
# New 第 3 轮，帧中位数。47 个点，排除 14 条观测。
# 验证 RMSE 11.8 mm，验证中位数 9.0 mm，最大误差 38.8 mm。
# 和推荐的第 14 张最近：旋转差 0.12 deg，平移差 0.6 mm。
rotation_matrix = np.array([
    [0.030029238357589534, -0.41108146185533917, 0.911103878030678],
    [-0.9995487705619079, -0.011705128396734016, 0.027663066305000074],
    [-0.0007071858609491706, -0.9115234619515248, -0.411247465888928],
], dtype=np.float64)
translation_vector = np.array(
    [-73.13864265208457, -3.3567394702801607, 328.60661812871706],
    dtype=np.float64,
)
HT_NEW_RUN3_MEDIAN_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 13/15  HT_Old_Combined_CleanedSensitivity
# Old 三轮合成：每个物理点先取该轮帧中位数，三轮再取中位数。
# 45 个物理点，117 条观测，排除 66 条。组内验证 RMSE 12.8 mm，验证中位数 10.3 mm。
# 固定误差约 11 mm。用它预测清洗后的 New（51 个点）时，RMSE 12.3 mm、中位数 11.2 mm。
# 中位数和 RMSE 接近，误差铺在整组点上。相对第 14 张旋转差 0.92 deg，平移差 11.5 mm。
# 表现不是最好，但这约 11 mm 是稳定的。
rotation_matrix = np.array([
    [0.03443300496321705, -0.40300118853177264, 0.9145514803504405],
    [-0.9993194174156124, -0.001768391105798325, 0.036845281502768],
    [-0.013231407533756729, -0.9151977463012437, -0.4027878039610931],
], dtype=np.float64)
translation_vector = np.array(
    [-77.89306959530296, -12.066195932959022, 323.9070493801255],
    dtype=np.float64,
)
HT_OLD_COMBINED_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 14/15  HT_New_Combined_CleanedSensitivity
# New 三轮合成，规则与第 13 张相同。51 个物理点，139 条观测，排除 44 条。
# 组内验证 RMSE 11.9 mm，验证中位数 9.4 mm。
# 预测清洗后的 Old（45 个点），中位数误差 10.2 mm，RMSE 13.7 mm。
# 最好，推荐作为后续实验的候选。
# 仍是未补偿实验矩阵：相机点是 ArUco 中心，机器人点是 robtarget.trans，没有加 [0, 35, 60] mm。
rotation_matrix = np.array([
    [0.028751001804470263, -0.4118593750289163, 0.9107937390518394],
    [-0.9995844925220032, -0.009973194141692548, 0.027043995787259904],
    [-0.0020548004206234084, -0.9111928394140305, -0.4119749837014728],
], dtype=np.float64)
translation_vector = np.array(
    [-73.1241608866211, -2.9796787604874524, 329.10318658634867],
    dtype=np.float64,
)
HT_NEW_COMBINED_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

# 15/15  HT_Global_CleanedSensitivity
# Old 与 New 的物理点合在一起。96 个物理点，256 条观测，排除 110 条。
# Old 的 s15 与 New 的 s15 没有合并。代表值仍是中位数。
# 验证 RMSE 12.5 mm，验证中位数 10.2 mm，最大误差 40.7 mm。
# 姿态最多。验证中位数 10.2 mm，第 14 张是 9.4 mm，推荐仍是第 14 张。
rotation_matrix = np.array([
    [0.028675104660934286, -0.4042490773704095, 0.914199333743936],
    [-0.9995105250125945, -0.00015157762610286483, 0.031283980137913475],
    [-0.012507947942432204, -0.9146489274412322, -0.4040555540626481],
], dtype=np.float64)
translation_vector = np.array(
    [-76.39485801339333, -7.350804558417579, 324.8936226360455],
    dtype=np.float64,
)
HT_GLOBAL_CLEANED = build_homogeneous(rotation_matrix, translation_vector)

MATRICES = {
    "HT_Old_Run1_Mean_CleanedSensitivity": HT_OLD_RUN1_MEAN_CLEANED,
    "HT_Old_Run1_Median_CleanedSensitivity": HT_OLD_RUN1_MEDIAN_CLEANED,
    "HT_Old_Run2_Mean_CleanedSensitivity": HT_OLD_RUN2_MEAN_CLEANED,
    "HT_Old_Run2_Median_CleanedSensitivity": HT_OLD_RUN2_MEDIAN_CLEANED,
    "HT_Old_Run3_Mean_CleanedSensitivity": HT_OLD_RUN3_MEAN_CLEANED,
    "HT_Old_Run3_Median_CleanedSensitivity": HT_OLD_RUN3_MEDIAN_CLEANED,
    "HT_New_Run1_Mean_CleanedSensitivity": HT_NEW_RUN1_MEAN_CLEANED,
    "HT_New_Run1_Median_CleanedSensitivity": HT_NEW_RUN1_MEDIAN_CLEANED,
    "HT_New_Run2_Mean_CleanedSensitivity": HT_NEW_RUN2_MEAN_CLEANED,
    "HT_New_Run2_Median_CleanedSensitivity": HT_NEW_RUN2_MEDIAN_CLEANED,
    "HT_New_Run3_Mean_CleanedSensitivity": HT_NEW_RUN3_MEAN_CLEANED,
    "HT_New_Run3_Median_CleanedSensitivity": HT_NEW_RUN3_MEDIAN_CLEANED,
    "HT_Old_Combined_CleanedSensitivity": HT_OLD_COMBINED_CLEANED,
    "HT_New_Combined_CleanedSensitivity": HT_NEW_COMBINED_CLEANED,
    "HT_Global_CleanedSensitivity": HT_GLOBAL_CLEANED,
}

BEST = "HT_New_Combined_CleanedSensitivity"
BEST_SINGLE_RUN = "HT_New_Run2_Mean_CleanedSensitivity"
LOOSEST = "HT_Old_Run1_Median_CleanedSensitivity"
FIXED_BIAS = "HT_Old_Combined_CleanedSensitivity"
