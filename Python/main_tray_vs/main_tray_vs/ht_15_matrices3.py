"""剔除超出臂展的相机点之后的 15 张未补偿 HT。

上一版在 HT_Analysis_Results，没有删除。
p_B = R @ p_C + t，单位 mm。
相机点是 ArUco 中心，机器人点是 RAPID robtarget.trans。
没有加入法兰到标记的 [0, 35, 60] mm。

最好：HT_NEW_COMBINED（第 14 张）。
单轮最好：HT_NEW_RUN3_MEDIAN（第 12 张）。
最差：HT_OLD_RUN2_MEDIAN（第 4 张）。六张 Old 单轮的验证 RMSE 都在 50 mm 左右。
固定误差：HT_OLD_COMBINED（第 13 张），预测 New 时整组约 17 mm。
old:s26 的相机深度约 640 mm，在臂展内，三轮都差约 325 mm，这次没有按残差剔除。
"""

import numpy as np


def build_homogeneous(rotation_matrix, translation_vector):
    """把 3x3 旋转和 3x1 平移合成 4x4。平移单位 mm。"""
    homogeneous = np.eye(4, dtype=np.float64)
    homogeneous[:3, :3] = np.asarray(rotation_matrix, dtype=np.float64)
    homogeneous[:3, 3] = np.asarray(translation_vector, dtype=np.float64).reshape(3)
    return homogeneous

# 1/15  HT_Old_Run1_Mean
# Old 第 1 轮，窗口平均数。51 个点，含 15 个 WARNING；本轮剔除 10 个，其中 s48 的相机均值 Z 为 9213 mm。
# 验证 RMSE 50.8 mm，中位数 15.8 mm，最大 338 mm。最大来自 old:s26，三轮相机深度都约 640 mm，不是数米级野点。
rotation_matrix = np.array([
    [0.041370138070609, -0.435103121568545, 0.899429700019588],
    [-0.998749751954631, 0.007273974192482233, 0.04945727721993019],
    [-0.028061444128567307, -0.9003512441823772, -0.4342582094246053],
], dtype=np.float64)
translation_vector = np.array(
    [-78.73738290275645, -19.564960739528505, 344.695639162204],
    dtype=np.float64,
)
HT_OLD_RUN1_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 2/15  HT_Old_Run1_Median
# 同一轮改用帧中位数。s48 的中位数 Z 是 338 mm，但窗口标准差超过 1 万 mm，整点已剔除。
# 验证 RMSE 49.9 mm，中位数 13.8 mm。中位数地板约 14 mm，不是最好；最大仍是 s26 约 326 mm。
rotation_matrix = np.array([
    [0.04327761641410869, -0.4302868001061426, 0.9016542117530031],
    [-0.9986683424985837, 0.0067357337024059605, 0.05114852473552943],
    [-0.028081837700621837, -0.902667103391891, -0.4294222989615808],
], dtype=np.float64)
translation_vector = np.array(
    [-78.44826360388225, -20.462415208316052, 341.42365735932367],
    dtype=np.float64,
)
HT_OLD_RUN1_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 3/15  HT_Old_Run2_Mean
# Old 第 2 轮，窗口平均数。51 个点。s48 均值 Z 8638 mm 已剔除。
# 验证 RMSE 52.4 mm，中位数 19.0 mm，最大 326 mm，同样是 s26。
rotation_matrix = np.array([
    [0.04444108356977973, -0.43680868188807814, 0.8984559897503849],
    [-0.9985918991427213, 0.006655694443432219, 0.05263003608215611],
    [-0.02896910522741861, -0.8995298089327975, -0.43589782493566914],
], dtype=np.float64)
translation_vector = np.array(
    [-79.67658396834952, -20.749497420895455, 346.2680309761387],
    dtype=np.float64,
)
HT_OLD_RUN2_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 4/15  HT_Old_Run2_Median
# 同一轮的帧中位数。s52 的 5×5 深度极差 1030 mm，整点已剔除。
# 最差。六张 Old 单轮里验证 RMSE 最高，53.2 mm；中位数 16.9 mm。最大 335 mm 仍是 s26，另有 s21 约 114 mm。
rotation_matrix = np.array([
    [0.0361388128439924, -0.44016514426830006, 0.8971892955098681],
    [-0.9990590638410429, 0.005628614263390803, 0.04300355402211484],
    [-0.023978598025862274, -0.897899195050725, -0.4395475655307098],
], dtype=np.float64)
translation_vector = np.array(
    [-77.34750356520811, -15.283666778086188, 347.4141711321111],
    dtype=np.float64,
)
HT_OLD_RUN2_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 5/15  HT_Old_Run3_Mean
# Old 第 3 轮，窗口平均数。49 个点。s48 均值 Z 3908 mm、s49 均值 Z 6422 mm 已剔除。
# 验证 RMSE 52.0 mm，中位数 17.4 mm。上一版这张相对 Global 转了 169 deg，剔除数米点之后旋转已回到与其他 Old 矩阵同一类。
rotation_matrix = np.array([
    [0.041839570574748276, -0.4318435484439938, 0.9009775801879982],
    [-0.9987856752415422, 0.005399492311730176, 0.04896958663366869],
    [-0.026012021574985483, -0.901932367281536, -0.43109323769169344],
], dtype=np.float64)
translation_vector = np.array(
    [-79.80790842671041, -18.574917272688978, 343.006172804293],
    dtype=np.float64,
)
HT_OLD_RUN3_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 6/15  HT_Old_Run3_Median
# 同一轮的帧中位数。s52 深度极差 1002 mm，整点已剔除。
# 验证 RMSE 52.5 mm，中位数 15.7 mm。固定地板约 16 mm，不是最好；最大 339 mm 仍是 s26。
rotation_matrix = np.array([
    [0.040295327461397606, -0.4332825453020441, 0.900356886196446],
    [-0.9988629808744173, 0.0055071197455586465, 0.04735416635080718],
    [-0.025476106913156972, -0.9012413148367587, -0.43256798356773835],
], dtype=np.float64)
translation_vector = np.array(
    [-77.47341343124128, -17.619411536360985, 342.7787606363772],
    dtype=np.float64,
)
HT_OLD_RUN3_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 7/15  HT_New_Run1_Mean
# New 第 1 轮，窗口平均数。58 个点。New 没有相机距离超过 1000 mm 的点。
# 验证 RMSE 16.3 mm，中位数 9.9 mm。误差地板约 10 mm，稳定，但不是第一。
rotation_matrix = np.array([
    [0.004050474610010031, -0.395210887150983, 0.9185814870400812],
    [-0.9999752900301533, -0.00687875190348592, 0.0014498625319872593],
    [0.00574569269497015, -0.9185646615506081, -0.39522898371181187],
], dtype=np.float64)
translation_vector = np.array(
    [-80.68928353402373, 8.096977604701195, 322.5271910551188],
    dtype=np.float64,
)
HT_NEW_RUN1_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 8/15  HT_New_Run1_Median
# 同一轮的帧中位数。
# 验证 RMSE 17.7 mm，中位数 9.7 mm。地板约 10 mm，不是第一。
rotation_matrix = np.array([
    [0.009327009202570978, -0.3931973418446285, 0.9194067963995338],
    [-0.9999551614672457, -0.005173323322659086, 0.00793169470004865],
    [0.001637667350442645, -0.9194395505372375, -0.39322796308697566],
], dtype=np.float64)
translation_vector = np.array(
    [-80.13709099126663, 4.625590098452065, 320.88348177353],
    dtype=np.float64,
)
HT_NEW_RUN1_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 9/15  HT_New_Run2_Mean
# New 第 2 轮，窗口平均数。60 个点。
# 验证 RMSE 20.5 mm，中位数 10.6 mm。地板约 10 mm，不是第一。
rotation_matrix = np.array([
    [-0.002597731108439694, -0.3876317471376878, 0.9218106532276962],
    [-0.9999744383587065, -0.005133658742591244, -0.004976763717912172],
    [0.00666141293402215, -0.9218000185283671, -0.3876085027689978],
], dtype=np.float64)
translation_vector = np.array(
    [-84.04769438994447, 10.906799897103987, 319.0687334399627],
    dtype=np.float64,
)
HT_NEW_RUN2_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 10/15  HT_New_Run2_Median
# 同一轮的帧中位数。
# 验证 RMSE 21.3 mm，中位数 10.8 mm，最大 121 mm（s44）。地板约 10 mm，不是第一。
rotation_matrix = np.array([
    [-0.0005869010895230238, -0.3886063414818832, 0.9214036937776934],
    [-0.9999860674062694, -0.004605617204006339, -0.0025793959590496075],
    [0.005246002330777363, -0.9213923700846649, -0.3885982241458513],
], dtype=np.float64)
translation_vector = np.array(
    [-82.83560926848418, 9.611510893627841, 319.0308908904267],
    dtype=np.float64,
)
HT_NEW_RUN2_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 11/15  HT_New_Run3_Mean
# New 第 3 轮，窗口平均数。60 个点。
# 验证 RMSE 16.2 mm，中位数 10.1 mm。地板约 10 mm，不是第一。
rotation_matrix = np.array([
    [0.0036179942785930395, -0.3948560259703314, 0.9187358863527194],
    [-0.9999887469951038, -0.004247813592985132, 0.0021123358733715313],
    [0.003068550237938367, -0.9187331902323959, -0.39486695121881377],
], dtype=np.float64)
translation_vector = np.array(
    [-80.94058343645128, 7.330829145396862, 322.113730934712],
    dtype=np.float64,
)
HT_NEW_RUN3_MEAN = build_homogeneous(rotation_matrix, translation_vector)

# 12/15  HT_New_Run3_Median
# 同一轮的帧中位数。
# 单轮最好。验证 RMSE 15.5 mm，中位数 10.1 mm，最大 50 mm。仍不如三轮合并的第 14 张。
rotation_matrix = np.array([
    [0.005650500498997963, -0.3973578849441982, 0.9176463279045931],
    [-0.9999820525749135, -0.004072922431638321, 0.004393840111901713],
    [0.0019915752995849838, -0.9176546859116119, -0.3973737674435579],
], dtype=np.float64)
translation_vector = np.array(
    [-79.34284113735981, 6.146410541936348, 322.8155589118912],
    dtype=np.float64,
)
HT_NEW_RUN3_MEDIAN = build_homogeneous(rotation_matrix, translation_vector)

# 13/15  HT_Old_Combined
# Old 三轮按物理点合并：每轮先取帧中位数，再对三轮取中位数。53 个物理点。s48、s49、s52 的不可信采样没有进入。
# 固定误差。预测全部 New 点时 RMSE 21.7 mm，中位数 17.0 mm，两者接近，整组大约差 17 mm，不是个别 New 坏点。不是最好。组内最大仍是 old:s26 约 325 mm。
rotation_matrix = np.array([
    [0.04027449003104459, -0.43511413822168016, 0.8994740975549791],
    [-0.9988473346624197, 0.005995246651915948, 0.04762414361815893],
    [-0.026114507280812445, -0.9003553430380626, -0.4343711417351447],
], dtype=np.float64)
translation_vector = np.array(
    [-77.55477834617227, -17.959836790241326, 344.27916626714216],
    dtype=np.float64,
)
HT_OLD_COMBINED = build_homogeneous(rotation_matrix, translation_vector)

# 14/15  HT_New_Combined
# New 三轮按同样方式合并。61 个物理点。New 没有因臂展被剔除的点，只排除了原先失败的 S。
# 最好，推荐候选。验证 RMSE 17.6 mm，中位数 9.9 mm。预测清洗后的 Old 点时中位数约 10.2 mm。未加 [0, 35, 60]。
rotation_matrix = np.array([
    [0.002487476946309068, -0.39183751931518196, 0.9200310706249906],
    [-0.9999923300939482, -0.0037580378455482335, 0.0011031340930836553],
    [0.003025262255983282, -0.9200267580937393, -0.39184386199072546],
], dtype=np.float64)
translation_vector = np.array(
    [-81.46809799545474, 7.756143525855466, 320.382955603828],
    dtype=np.float64,
)
HT_NEW_COMBINED = build_homogeneous(rotation_matrix, translation_vector)

# 15/15  HT_Global
# 六次采样合成一张。114 个物理点。Old 的 s48、s49、s52 不在里面。
# 不是最好。验证中位数 11.4 mm，高于第 14 张的 9.9 mm；RMSE 36.4 mm 主要被 old:s26 拉高。点更多不等于更准。
rotation_matrix = np.array([
    [0.023884083591317433, -0.40662063613029203, 0.9132848453927174],
    [-0.9996599037846001, -0.0001461695393998341, 0.02607787184243174],
    [-0.010470306412301681, -0.9135970867441855, -0.40648583711633807],
], dtype=np.float64)
translation_vector = np.array(
    [-80.30768051214801, -5.914671192203199, 328.43014739186464],
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
WORST = "HT_Old_Run2_Median"
FIXED_BIAS = "HT_Old_Combined"
