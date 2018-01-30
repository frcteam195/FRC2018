package org.usfirst.frc.team195.robot.Autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.CustomAuto;

public class AutoProfileTest2 implements CustomAuto {
	private DriveBaseSubsystem driveBaseSubsystem;

	 public AutoProfileTest2() {
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
	}

	@Override
	public void start() {
		driveBaseSubsystem.setMotionProfileTrajectory(kMotionProfileLeft, kMotionProfileRight);
		driveBaseSubsystem.setControlMode(ControlMode.MotionProfile);
		driveBaseSubsystem.setBrakeMode(true);
		driveBaseSubsystem.setGear(false);
		driveBaseSubsystem.startMPTrajectory();
	}

	public static double[][] kMotionProfileLeft = new double[][] {{2.1742478564466578E-5,	1.7811438440011018,	10.0},
			{1.08709511223944E-4,	3.5621696577321944,	10.0},
			{3.0437128812951927E-4,	8.014306382052363,	10.0},
			{6.521664340886042E-4,	14.245689178484117,	10.0},
			{0.0011954733568750452,	22.253851557332617,	10.0},
			{0.001977571957587796,	32.03475868519427,	10.0},
			{0.003041595904261836,	43.58242085576867,	10.0},
			{0.004430475609620879,	56.8885127315064,	10.0},
			{0.0061868720909334495,	71.94199987456295,	10.0},
			{0.008331451462349551,	87.84197105320354,	10.0},
			{0.010862942978465217,	103.68989250009767,	10.0},
			{0.01377984991135867,	119.47650797131581,	10.0},
			{0.01708045186407194,	135.19265598313564,	10.0},
			{0.0207628073666756,	150.829281386646,	10.0},
			{0.02482475656295857,	166.37743907975013,	10.0},
			{0.02926392411046974,	181.82830274605755,	10.0},
			{0.034077721984193295,	197.17316090771678,	10.0},
			{0.03926335238553102,	212.40342123879327,	10.0},
			{0.044817810388423154,	227.51059979846167,	10.0},
			{0.050737886582084446,	242.48632089236625,	10.0},
			{0.057020169276517096,	257.32229916396176,	10.0},
			{0.06366104639673727,	272.010326844218,	10.0},
			{0.07065670702750738,	286.54225943634407,	10.0},
			{0.07800314226932419,	300.9099875048161,	10.0},
			{0.08569614559830531,	315.10541635506706,	10.0},
			{0.0937313124942715,	329.12043605877443,	10.0},
			{0.10210403911183905,	342.94688225556706,	10.0},
			{0.11080952030676339,	356.5765097441016,	10.0},
			{0.11984274638066005,	370.00093998680677,	10.0},
			{0.12919849905078282,	383.21162936822907,	10.0},
			{0.13887134580930055,	396.1998032288861,	10.0},
			{0.14885563344219158,	408.95642144321636,	10.0},
			{0.15914547982682709,	421.47210791467023,	10.0},
			{0.1697347642336892,	433.73708930507223,	10.0},
			{0.18061711623100107,	445.74113780989427,	10.0},
			{0.19178590280365285,	457.47349801581663,	10.0},
			{0.2032342131918944,	468.9227935023734,	10.0},
			{0.21495484256328928,	480.07697905233516,	10.0},
			{0.22694027262482908,	490.92321532067103,	10.0},
			{0.239182650898791,	501.4478141014786,	10.0},
			{0.25167376695238886,	511.6361135553682,	10.0},
			{0.2644050263475512,	521.4723848258482,	10.0},
			{0.27736742197178915,	530.9397247687887,	10.0},
			{0.2905515022320527,	540.0199274603963,	10.0},
			{0.30394733636509447,	548.6933660893892,	10.0},
			{0.3175444766440986,	556.9388658280117,	10.0},
			{0.3313319170702361,	564.7335598545919,	10.0},
			{0.3452980483760418,	572.0527382857965,	10.0},
			{0.35943060957997475,	578.8697069130932,	10.0},
			{0.3737166354296973,	585.1556188046376,	10.0},
			{0.3881424001497643,	590.8793229339445,	10.0},
			{0.40269335665701067,	596.007178536811,	10.0},
			{0.4173540721377323,	600.502906090356,	10.0},
			{0.4321081595107274,	604.3274187978814,	10.0},
			{0.44693820464004835,	607.438648496985,	10.0},
			{0.46182569112472527,	609.7914464123681,	10.0},
			{0.4767509205872927,	611.3373987867596,	10.0},
			{0.49169293235151895,	612.0248018627061,	10.0},
			{0.5066294212235368,	611.7985841978552,	10.0},
			{0.5215366564460947,	610.6003547159713,	10.0},
			{0.5363894030590688,	608.3685012674172,	10.0},
			{0.5511608497345635,	605.0384558282616,	10.0},
			{0.56582254592209,	600.5430758410926,	10.0},
			{0.580344353421402,	594.8132351718143,	10.0},
			{0.5946944200651328,	587.7787297272176,	10.0},
			{0.6088391812106922,	579.36941652212,	10.0},
			{0.6227434019615004,	569.5168819530921,	10.0},
			{0.636370270147567,	558.1565209012984,	10.0},
			{0.6496815566917153,	545.230296848313,	10.0},
			{0.662637860816018,	530.6902169314376,	10.0},
			{0.6751989614401962,	514.5026815663423,	10.0},
			{0.6873242982787229,	496.65379690605135,	10.0},
			{0.6989736101366806,	477.15581370194764,	10.0},
			{0.7101077570921663,	456.0546592966959,	10.0},
			{0.7206897545203357,	433.4386146578198,	10.0},
			{0.7306860425586567,	409.44795804962536,	10.0},
			{0.7400680069862486,	384.2852629541703,	10.0},
			{0.7488137541338955,	358.2258031676117,	10.0},
			{0.7569101217023348,	331.6272156032768,	10.0},
			{0.7643548789191922,	304.93725560248095,	10.0},
			{0.771159033481501,	278.6981708721629,	10.0},
			{0.7773491195916163,	253.545927070323,	10.0},
			{0.7829692978832834,	230.20250282668232,	10.0},
			{0.7880830586569045,	209.45964128753036,	10.0},
			{0.7927742958528838,	192.15307554731382,	10.0},
			{0.7971475204848358,	179.12728092474902,	10.0},
			{0.8013270191143574,	171.19226386520577,	10.0},
			{0.8054548393049876,	169.075515008211,	10.0},
			{0.8096875972488993,	173.37376538262063,	10.0},
			{0.8141922386624735,	184.51011229999904,	10.0},
			{0.8191410204204828,	202.70210080805975,	10.0},
			{0.8247060900052237,	227.94525019098256,	10.0},
			{0.8310540989425219,	260.01444607173875,	10.0},
			{0.8383412812795282,	298.4829885237769,	10.0},
			{0.8467093600410863,	342.7565060734148,	10.0},
			{0.8562825292684855,	392.1170115542793,	10.0},
			{0.8671656217061858,	445.771466248205,	10.0},
			{0.8794434381816576,	502.8993628353255,	10.0},
			{0.8931811078613603,	562.6949500806211,	10.0},
			{0.9084252773097191,	624.401180604778,	10.0},
			{0.925205896759402,	687.3341726590074,	10.0},
			{0.9435383740170536,	750.8982684734141,	10.0},
			{0.9634258946291402,	814.592844271061,	10.0},
			{0.9848617461401722,	878.0124778918658,	10.0},
			{1.0078315290637947,	940.842308551582,	10.0},
			{1.03231517877672,	1002.850292241423,	10.0},
			{1.0582887566818704,	1063.877750994958,	10.0},
			{1.085725997897116,	1123.8294001764648,	10.0},
			{1.1145996203878124,	1182.6635772189202,	10.0},
			{1.1448824148343395,	1240.3832605297478,	10.0},
			{1.1765481394683066,	1297.0280810072918,	10.0},
			{1.2095722500621422,	1352.6675699235084,	10.0},
			{1.2439324927645625,	1407.3955410911367,	10.0},
			{1.279609388849083,	1461.3256636219585,	10.0},
			{1.3165866357983338,	1514.588035041311,	10.0},
			{1.3548514483453242,	1567.3267219247268,	10.0},
			{1.3943948593924955,	1619.698116492135,	10.0},
			{1.4352119979501559,	1671.8699953217656,	10.0},
			{1.4773023597355452,	1724.021218729547,	10.0},
			{1.5206700832182296,	1776.3419538507537,	10.0},
			{1.565324242319049,	1829.034356769568,	10.0},
			{1.6112791652188203,	1882.3136419746186,	10.0},
			{1.6585547870700101,	1936.4094710247425,	10.0},
			{1.7071770424217192,	1991.5675792059978,	10.0},
			{1.7571783007778274,	2048.0515422661906,	10.0},
			{1.8085978457357388,	2106.144561476047,	10.0},
			{1.8614823934111275,	2166.1510727839227,	10.0},
			{1.9158866393932183,	2228.3979154264352,	10.0},
			{1.9718738147785542,	2293.2347037833583,	10.0},
			{2.029516216648915,	2361.0327806099826,	10.0},
			{2.0888956616880403,	2432.182068802579,	10.0},
			{2.1501037821739124,	2507.0846151013398,	10.0},
			{2.2132420482295885,	2586.1433776404765,	10.0},
			{2.2784213515644907,	2669.7442645976034,	10.0},
			{2.3457609219075115,	2758.228801250118,	10.0},
			{2.415386273964176,	2851.854420240981,	10.0},
			{2.4874257970484255,	2950.738865530846,	10.0},
			{2.5620055270763666,	3054.7857419444813,	10.0},
			{2.6392415950036527,	3163.589342301666,	10.0},
			{2.71922989277043,	3276.3206765271884,	10.0},
			{2.8019985427274507,	3390.2039022395497,	10.0},
			{2.8874872261936817,	3501.6164747768375,	10.0},
			{2.975561198904269,	3607.50992222566,	10.0},
			{3.0659952242017945,	3704.1776761866354,	10.0},
			{3.1584622926248525,	3787.4511226084605,	10.0},
			{3.2525310261355798,	3853.0553245993788,	10.0},
			{3.34767507065485,	3897.1000635093087,	10.0},
			{3.44329577652382,	3916.6241123930185,	10.0},
			{3.538756326950518,	3910.0641454775678,	10.0},
			{3.6334595762439545,	3879.0450910591653,	10.0},
			{3.7268713380254574,	3826.145762570354,	10.0},
			{3.8184897484303533,	3752.6900901845393,	10.0},
			{3.9078809632684157,	3661.4641597670393,	10.0},
			{3.9947110211092323,	3556.559169159844,	10.0},
			{4.078742057122791,	3441.9112351153835,	10.0},
			{4.159820147047812,	3320.958563328868,	10.0},
			{4.237858624538216,	3196.4560380069297,	10.0},
			{4.3128201618703255,	3070.424569123203,	10.0},
			{4.384699885246724,	2944.193469497275,	10.0},
			{4.453520934642422,	2818.9101832477722,	10.0},
			{4.519337581741538,	2695.8498651798013,	10.0},
			{4.582211360012962,	2575.3099579975637,	10.0},
			{4.642184955684814,	2456.5184787190396,	10.0},
			{4.699276298933408,	2338.461419462383,	10.0},
			{4.753473304832669,	2219.9093616337777,	10.0},
			{4.804767759907693,	2101.020879872951,	10.0},
			{4.854196915637916,	2024.6182187099398,	10.0},
			{4.9023494524453195,	1972.3279076312028,	10.0},
			{4.949352252979132,	1925.2347098650077,	10.0},
			{4.995323557952516,	1882.9846517097815,	10.0},
			{5.04037333475783,	1845.238857945657,	10.0},
			{5.0846036222755675,	1811.6725767265207,	10.0},
			{5.128108847518696,	1781.9740259585762,	10.0},
			{5.170976117114901,	1755.8433626605713,	10.0},
			{5.213285490356544,	1732.9919279776748,	10.0},
			{5.255110241785484,	1713.1418185294124,	10.0},
			{5.296517120874175,	1696.0257674727538,	10.0},
			{5.337566614984779,	1681.387278770341,	10.0},
			{5.378313219902851,	1668.9809374442254,	10.0},
			{5.418805720281401,	1658.5728155053926,	10.0},
			{5.459087480423183,	1649.9408954074527,	10.0},
			{5.499196744295527,	1642.8754482111945,	10.0},
			{5.539166942412179,	1637.1793148580784,	10.0},
			{5.5790270024120465,	1632.6680575945866,	10.0},
			{5.618801659728571,	1629.169963684827,	10.0},
			{5.658511764592847,	1626.525895240754,	10.0},
			{5.698174581769451,	1624.5889915537282,	10.0},
			{5.7378040797999885,	1623.2242393308024,	10.0},
			{5.777411206981974,	1622.3079293740907,	10.0},
			{5.8170041518860245,	1621.7270232699368,	10.0},
			{5.8565885868316325,	1621.378455372117,	10.0},
			{5.896167893299954,	1621.1683929424294,	10.0},
			{5.93574336883262,	1621.0114778180216,	10.0},
			{5.975314415456429,	1620.8300697112102,	10.0},
			{6.014878710099724,	1620.5535085893653,	10.0},
			{6.054432357833145,	1620.1174111608775,	10.0},
			{6.0939700290483705,	1619.4630129756147,	10.0},
			{6.133485081911528,	1618.536565274954,	10.0},
			{6.172969671588098,	1617.2887931523496,	10.0},
			{6.212414847824807,	1615.6744186555827,	10.0},
			{6.25181064254096,	1613.6517515736352,	10.0},
			{6.291146149067493,	1611.1823473268005,	10.0},
			{6.330409594649905,	1608.2307310555818,	10.0},
			{6.369588407755963,	1604.7641848241342,	10.0},
			{6.408669281641861,	1600.7525943663807,	10.0},
			{6.447638235475844,	1596.1683490399253,	10.0},
			{6.486480674226523,	1590.9862912278074,	10.0},
			{6.525181448316624,	1585.1837067304812,	10.0},
			{6.563724913864728,	1578.7403488503046,	10.0},
			{6.602094994212475,	1571.6384910437553,	10.0},
			{6.640275243158668,	1563.8629968361142,	10.0},
			{6.678248910207856,	1555.4014023347368,	10.0},
			{6.715999007880634,	1546.244000676958,	10.0},
			{6.753508381044306,	1536.3839247840713,	10.0},
			{6.79075977795095,	1525.8172172960763,	10.0},
			{6.827735922603966,	1514.5428849875454,	10.0},
			{6.864419587905777,	1502.562930762125,	10.0},
			{6.900793668935026,	1489.8823589580377,	10.0},
			{6.936841255650456,	1476.5091518639792,	10.0},
			{6.972545704234543,	1462.4542140042386,	10.0},
			{7.007890706295701,	1447.73128442503,	10.0},
			{7.042860355148856,	1432.3568170252354,	10.0},
			{7.077439208428833,	1416.349830347915,	10.0},
			{7.111612346338783,	1399.73172879148,	10.0},
			{7.145365424952965,	1382.526100036919,	10.0},
			{7.178684724043512,	1364.7584907487985,	10.0},
			{7.21155718901998,	1346.456165436099,	10.0},
			{7.24397046675342,	1327.6478559617515,	10.0},
			{7.275912935027791,	1308.363500518233,	10.0},
			{7.307373725684276,	1288.6339852896394,	10.0},
			{7.3383427413966915,	1268.4908835805688,	10.0},
			{7.368810666382979,	1247.9662074382559,	10.0},
			{7.3987689711766516,	1227.0921643488457,	10.0},
			{7.428209911882177,	1205.900931298288,	10.0},
			{7.4571265243344484,	1184.4244460450116,	10.0},
			{7.485512613485254,	1162.6942116169982,	10.0},
			{7.513362738680631,	1140.7411280026797,	10.0},
			{7.54067219515688,	1118.5953372671675,	10.0},
			{7.567436992396384,	1096.2860949301025,	10.0},
			{7.593653829781538,	1073.8416592959068,	10.0},
			{7.619320070044843,	1051.2892011849626,	10.0},
			{7.644433710977741,	1028.6547326115056,	10.0},
			{7.668993355867018,	1005.9630546648193,	10.0},
			{7.692998182992973,	983.2377190790752,	10.0},
			{7.716447914593885,	960.5010063733756,	10.0},
			{7.73934278561754,	937.7739171288885,	10.0},
			{7.7616835124962655,	915.0761729526399,	10.0},
			{7.783471262215764,	892.4262285106544,	10.0},
			{7.804707621878419,	869.8412917823136,	10.0},
			{7.825394568876319,	847.3373490340151,	10.0},
			{7.845534441848131,	824.9291969254695,	10.0},
			{7.865129912474335,	802.6304768492697,	10.0},
			{7.884183958189103,	780.4537124769503,	10.0},
			{7.902699835864919,	758.4103496013759,	10.0},
			{7.920681056448632,	736.5107951088565,	10.0},
			{7.938131360555471,	714.7644562161668,	10.0},
			{7.955054695039718,	693.1797804747671,	10.0},
			{7.971455190466462,	671.7642926794696,	10.0},
			{7.987337139487526,	650.5246319027718,	10.0},
			{8.002704976016812,	629.4665842396222,	10.0},
			{8.017563255214965,	608.5951159563165,	10.0},
			{8.031916634217081,	587.9144039267532,	10.0},
			{8.04576985351271,	567.4278623488694,	10.0},
			{8.059127718957821,	547.1381686318109,	10.0},
			{8.071995084361667,	527.0472869415651,	10.0},
			{8.084376834630747,	507.1564910214407,	10.0},
			{8.096277869324087,	487.4663810392328,	10.0},
			{8.107703086744445,	467.97690553787135,	10.0},
			{8.11865736836516,	448.687375184426,	10.0},
			{8.129145563683704,	429.59648024758235,	10.0},
			{8.139172475429202,	410.7023050956556,	10.0},
			{8.14874284511478,	392.0023423212899,	10.0},
			{8.157861338931994,	373.4935067529892,	10.0},
			{8.166532533987217,	355.17214946195685,	10.0},
			{8.174760904866094,	337.03407119873987,	10.0},
			{8.182550810550506,	319.0745368335853,	10.0},
			{8.189906481692999,	301.2882899964612,	10.0},
			{8.196832008266984,	283.6695684704624,	10.0},
			{8.203331327613606,	266.21212043762915,	10.0},
			{8.209408212907233,	248.90922162701682,	10.0},
			{8.215066262068232,	231.75369363448715,	10.0},
			{8.220308887150896,	214.73792338588626,	10.0},
			{8.225139304247103,	197.85388426068184,	10.0},
			{8.229560523913653,	181.09315754191303,	10.0},
			{8.233575342183764,	164.44695634372144,	10.0},
			{8.2371863321767,	147.90615011065108,	10.0},
			{8.240395836340742,	131.4612905591969,	10.0},
			{8.2432059593677,	115.10263918423406,	10.0},
			{8.245618561794815,	98.82019541464315,	10.0},
			{8.247635254332696,	82.60372635157772,	10.0},
			{8.249265381546808,	66.77001068998507,	10.0},
			{8.250539782394224,	52.199458710182974,	10.0},
			{8.251502767520716,	39.44387078108104,	10.0},
			{8.252198367841112,	28.491789123438398,	10.0},
			{8.252670406006894,	19.334683270367556,	10.0},
			{8.252962557671127,	11.96653216698972,	10.0},
			{8.253118402212325,	6.383392407430498,	10.0},
			{8.253181462684704,	2.5829569487178405,	10.0},
			{8.253195234848466,	0.56410782770597,	10.0},
			{8.253195234848466,	0.0,	10.0},
	};
	public static double[][] kMotionProfileRight = new double[][] {{2.1742478564466578E-5,	1.7811438440011018,	10.0},
			{1.0871527448883192E-4,	3.562405721062004,	10.0},
			{3.0441811186331834E-4,	8.015988218858963,	10.0},
			{6.52382280422584E-4,	14.252612344187515,	10.0},
			{0.0011961992859530922,	22.27474454652961,	10.0},
			{0.0019795591431243397,	32.0864197497343,	10.0},
			{0.0030462980968244267,	43.693627543555586,	10.0},
			{0.004440455648020634,	57.10469329699664,	10.0},
			{0.006206340693955405,	72.33065148148819,	10.0},
			{0.008366772083184522,	88.49126970282458,	10.0},
			{0.010923020548555644,	104.70393714160119,	10.0},
			{0.013876582832771607,	120.9779111614858,	10.0},
			{0.017229179318562327,	137.32235205798773,	10.0},
			{0.020982751496253952,	153.74631639824904,	10.0},
			{0.02513945918703515,	170.25874701439778,	10.0},
			{0.029701677771722775,	186.86847322880507,	10.0},
			{0.03467199524440087,	203.58420368089483,	10.0},
			{0.04005320944261927,	220.4145335590255,	10.0},
			{0.04584832522923951,	237.36794261996528,	10.0},
			{0.052060552055957776,	254.45281082237963,	10.0},
			{0.058693301626767606,	271.6774224203706,	10.0},
			{0.06575018596410807,	289.0499824574658,	10.0},
			{0.07323501601941601,	306.57863906541286,	10.0},
			{0.08115180065163691,	324.2714985357678,	10.0},
			{0.08950474636464337,	342.13665640474454,	10.0},
			{0.09829825774476482,	360.1822261297745,	10.0},
			{0.10753693854485029,	378.4163655715011,	10.0},
			{0.11722559395418994,	396.84732556655274,	10.0},
			{0.1273692335570919,	415.4834781348637,	10.0},
			{0.13797307574543838,	434.33337603467146,	10.0},
			{0.1490425528611286,	453.40578265867185,	10.0},
			{0.16058331814965005,	472.70974621783796,	10.0},
			{0.17260125375277255,	492.25464230389827,	10.0},
			{0.18510248021523273,	512.0502359023698,	10.0},
			{0.1980933678670246,	532.1067582173943,	10.0},
			{0.21158054988407254,	552.4349754182828,	10.0},
			{0.22557093668228498,	573.0462432547815,	10.0},
			{0.24007173326707576,	593.9526281130304,	10.0},
			{0.2550904575345968,	615.1669459976607,	10.0},
			{0.2706349619360672,	636.7029002842268,	10.0},
			{0.2867134567274195,	658.5751466537918,	10.0},
			{0.3033345360855697,	680.7994105098331,	10.0},
			{0.32050720704255875,	703.392602398269,	10.0},
			{0.3382409209382816,	726.3729211688078,	10.0},
			{0.3565456081091573,	749.7599865190682,	10.0},
			{0.3754317159450963,	773.5749769600594,	10.0},
			{0.39491025017256864,	797.8407619572685,	10.0},
			{0.41499281951449135,	822.5820402451528,	10.0},
			{0.43569168445984235,	847.8255081615779,	10.0},
			{0.4570198095613504,	873.6000041577704,	10.0},
			{0.4789909201532456,	899.9366898440293,	10.0},
			{0.5016195624337836,	926.8691878108369,	10.0},
			{0.5249211682837651,	954.4337756152444,	10.0},
			{0.548912123960075,	982.6695445016541,	10.0},
			{0.573609841917876,	1011.6185275515268,	10.0},
			{0.599032837866367,	1041.3259140501914,	10.0},
			{0.6252008081695676,	1071.8400636190918,	10.0},
			{0.652134711878258,	1103.2126959079599,	10.0},
			{0.6798568520991423,	1135.498863447422,	10.0},
			{0.7083909575285975,	1168.7569583904865,	10.0},
			{0.7377622601707289,	1203.0485562217032,	10.0},
			{0.7679975684103154,	1238.438225493463,	10.0},
			{0.7991253296601346,	1274.993100792594,	10.0},
			{0.8311756776353689,	1312.7822530656003,	10.0},
			{0.8641804606507684,	1351.8759123107584,	10.0},
			{0.8981732376316629,	1392.3441451374474,	10.0},
			{0.9331892379433052,	1434.2553727648644,	10.0},
			{0.9692652671169261,	1477.6741549515136,	10.0},
			{1.0064395467583114,	1522.658494111141,	10.0},
			{1.044751469214777,	1569.2563438168384,	10.0},
			{1.0842412465810498,	1617.5012809225307,	10.0},
			{1.1249494283624364,	1667.4071257655962,	10.0},
			{1.1669162645256501,	1718.9616092452234,	10.0},
			{1.2101808826679081,	1772.1187591068904,	10.0},
			{1.2547802540873028,	1826.7902533384079,	10.0},
			{1.3007479245252955,	1882.8357811401847,	10.0},
			{1.348112493463532,	1940.0527437101591,	10.0},
			{1.396895839893013,	1998.1658697515475,	10.0},
			{1.44711111211036,	2056.817550022531,	10.0},
			{1.498760527523664,	2115.56005532893,	10.0},
			{1.5518330674173737,	2173.851234046351,	10.0},
			{1.6063021894367737,	2231.055237914636,	10.0},
			{1.662123728885925,	2286.4502558372396,	10.0},
			{1.7192341964396451,	2339.244751000378,	10.0},
			{1.7775497054616274,	2388.6032495403956,	10.0},
			{1.836965758906008,	2433.6815490818244,	10.0},
			{1.8973580915891919,	2473.6699467032086,	10.0},
			{1.9585846844180594,	2507.8412422704114,	10.0},
			{2.0204889561493378,	2535.5989701131675,	10.0},
			{2.082904001067918,	2556.520239865028,	10.0},
			{2.145657605673518,	2570.3876446453887,	10.0},
			{2.208577667069296,	2577.205714771057,	10.0},
			{2.2714975773036725,	2577.1995232000813,	10.0},
			{2.334261142994233,	2570.7956506853557,	10.0},
			{2.3967266770372095,	2558.588274400314,	10.0},
			{2.458770015055372,	2541.2951252239204,	10.0},
			{2.520286347872202,	2519.7089921773886,	10.0},
			{2.5811908909256984,	2494.6500834711796,	10.0},
			{2.641418525225894,	2466.9239009360367,	10.0},
			{2.700922608104141,	2437.2872346929885,	10.0},
			{2.7596731886183914,	2406.4237778637125,	10.0},
			{2.8176548538608324,	2374.929008330371,	10.0},
			{2.87486441026227,	2343.303430202894,	10.0},
			{2.9313085598956885,	2311.952368984818,	10.0},
			{2.9870016901269936,	2281.1906142742464,	10.0},
			{3.041963853110183,	2251.250195791468,	10.0},
			{3.0962189742191955,	2222.289760625133,	10.0},
			{3.149793305360356,	2194.404603541948,	10.0},
			{3.202714115496801,	2167.6363831887843,	10.0},
			{3.2550086019522113,	2141.982165213608,	10.0},
			{3.306702994537731,	2117.4023203028987,	10.0},
			{3.3578218265759334,	2093.827360284748,	10.0},
			{3.408387341239861,	2071.1634806344805,	10.0},
			{3.458419007517436,	2049.2970507294754,	10.0},
			{3.5079331185183076,	2028.097986595704,	10.0},
			{3.556942450471176,	2007.422236789475,	10.0},
			{3.60545596197207,	1987.1134310766258,	10.0},
			{3.65347851555009,	1967.0037945557076,	10.0},
			{3.7010106070632833,	1946.914468380382,	10.0},
			{3.74804808952255,	1926.655281531575,	10.0},
			{3.7945818801991753,	1906.0240661145817,	10.0},
			{3.8405976415539604,	1884.8055850920005,	10.0},
			{3.8860754282941268,	1862.7701448772007,	10.0},
			{3.9309892946815617,	1839.6719672293277,	10.0},
			{3.9753068585514093,	1815.247416108958,	10.0},
			{4.018988821852089,	1789.2132167958262,	10.0},
			{4.061988451749227,	1761.2648405867742,	10.0},
			{4.104251032845848,	1731.075321717596,	10.0},
			{4.145713310968195,	1698.2949118913048,	10.0},
			{4.186302961154189,	1662.552071618307,	10.0},
			{4.225938133673114,	1623.4566663751446,	10.0},
			{4.26452715679605,	1580.6063871154868,	10.0},
			{4.30196851300546,	1533.5979503374542,	10.0},
			{4.338151254346671,	1482.0450853359819,	10.0},
			{4.372956084292797,	1425.6058345933056,	10.0},
			{4.4062574102583225,	1364.022311547951,	10.0},
			{4.4379267524389885,	1297.1762557200593,	10.0},
			{4.467837973000861,	1225.1635942142925,	10.0},
			{4.495874828758864,	1148.3896118478483,	10.0},
			{4.521941310242608,	1067.6830815741146,	10.0},
			{4.545965701804239,	984.0390783644433,	10.0},
			{4.56792127049873,	899.3000937263379,	10.0},
			{4.587855747463536,	816.5161764784086,	10.0},
			{4.605907413649983,	739.396246996876,	10.0},
			{4.622316397029972,	672.1119592443274,	10.0},
			{4.637427278881678,	618.9417206459275,	10.0},
			{4.651679699215724,	583.7791368825112,	10.0},
			{4.665585659204031,	569.5881211210434,	10.0},
			{4.679695361165625,	577.9333923468605,	10.0},
			{4.694562801995407,	608.9703763879072,	10.0},
			{4.710706513890637,	661.2464392285898,	10.0},
			{4.728569167139246,	731.6542770630037,	10.0},
			{4.7484977921964715,	816.2764823439604,	10.0},
			{4.7707394512603996,	911.0183552584723,	10.0},
			{4.795445036519861,	1011.9407722275624,	10.0},
			{4.822681443050554,	1115.6032114971968,	10.0},
			{4.852448270743092,	1219.249262286339,	10.0},
			{4.884695759802716,	1320.857151882188,	10.0},
			{4.919341688700052,	1419.0972476348595,	10.0},
			{4.956291763159209,	1513.4750498471078,	10.0},
			{4.99546404745135,	1604.4967646060404,	10.0},
			{5.03679689843873,	1692.993576443137,	10.0},
			{5.080247530831234,	1779.7379027969298,	10.0},
			{5.125797924083761,	1865.7441076235034,	10.0},
			{5.173460077435064,	1952.2418012694056,	10.0},
			{5.223242119899278,	2039.0724593342022,	10.0},
			{5.274106733191566,	2083.4145604520972,	10.0},
			{5.325465222868385,	2103.6437371625375,	10.0},
			{5.377190700357395,	2118.675557949817,	10.0},
			{5.429164922526555,	2128.8641400488063,	10.0},
			{5.4812779221394825,	2134.548464145515,	10.0},
			{5.533427662282698,	2136.053356266084,	10.0},
			{5.585519719097171,	2133.690647120761,	10.0},
			{5.637466989799373,	2127.7602079622297,	10.0},
			{5.689189419237641,	2118.5507097914583,	10.0},
			{5.740613737003342,	2106.34005568309,	10.0},
			{5.791673197522301,	2091.3955028565424,	10.0},
			{5.8423073169432485,	2073.973531482045,	10.0},
			{5.892461602488941,	2054.3195359515607,	10.0},
			{5.942087271966481,	2032.6674218000417,	10.0},
			{5.991140962947785,	2009.23918259424,	10.0},
			{6.039584432774062,	1984.2445240843288,	10.0},
			{6.0873842517010335,	1957.8805832487103,	10.0},
			{6.134511492352119,	1930.3317770684546,	10.0},
			{6.180941419114839,	1901.769800200977,	10.0},
			{6.2266531812018755,	1872.3537750850078,	10.0},
			{6.271629512962496,	1842.2305489150745,	10.0},
			{6.315856444711014,	1811.5351244192811,	10.0},
			{6.359323026811952,	1780.391202854423,	10.0},
			{6.4020210692077715,	1748.9118165327488,	10.0},
			{6.443944898002042,	1717.200027413325,	10.0},
			{6.485091130090437,	1685.3496663406784,	10.0},
			{6.525458466302181,	1653.4460912331006,	10.0},
			{6.565047503012739,	1621.5669436643875,	10.0},
			{6.603860561754829,	1589.7828860760883,	10.0},
			{6.6419015360040765,	1558.1583052491894,	10.0},
			{6.679175754020398,	1526.7519699484906,	10.0},
			{6.71568985640952,	1495.6176338584366,	10.0},
			{6.751451686914783,	1464.8045774955738,	10.0},
			{6.786470194839071,	1434.3580845788495,	10.0},
			{6.820755347464132,	1404.319851522485,	10.0},
			{6.854318050813109,	1374.7283291741182,	10.0},
			{6.887170077148606,	1345.6189987019197,	10.0},
			{6.919323997661407,	1317.024584204364,	10.0},
			{6.950793118912116,	1288.9752064289858,	10.0},
			{6.981591421685548,	1261.498481599759,	10.0},
			{7.011733501105359,	1234.6195730354857,	10.0},
			{7.041234506980962,	1208.3612006646747,	10.0},
			{7.0701100835338435,	1182.7436156060116,	10.0},
			{7.098376307880779,	1157.7845492504634,	10.0},
			{7.126049626775941,	1133.4991419458263,	10.0},
			{7.153146791364384,	1109.899861542604,	10.0},
			{7.179684789833475,	1086.996417294002,	10.0},
			{7.205680778094374,	1064.79567916641,	10.0},
			{7.231152008717086,	1043.301606306309,	10.0},
			{7.256115758552641,	1022.5151932642741,	10.0},
			{7.280589255575608,	1002.4344380607491,	10.0},
			{7.3045896055851305,	983.0543363900629,	10.0},
			{7.328133719497176,	964.3669058374217,	10.0},
			{7.351238241986025,	946.3612411432599,	10.0},
			{7.373919482269964,	929.0236020301728,	10.0},
			{7.396193347822526,	912.3375330329529,	10.0},
			{7.418075281754743,	896.2840138636387,	10.0},
			{7.439580204547268,	880.8416375818313,	10.0},
			{7.460722460754367,	865.9868142427443,	10.0},
			{7.4815157711755385,	851.6939948511814,	10.0},
			{7.501973190888622,	837.9359114478995,	10.0},
			{7.522107073463253,	824.6838302568812,	10.0},
			{7.541929041462299,	811.9078092409414,	10.0},
			{7.561449963349247,	799.576960489406,	10.0},
			{7.58067993665857,	787.6597067498877,	10.0},
			{7.599628277346777,	776.1240345889706,	10.0},
			{7.6183035149807115,	764.9377334859577,	10.0},
			{7.636713393478905,	754.0686232860003,	10.0},
			{7.654864877016586,	743.4847657034505,	10.0},
			{7.672764160564757,	733.1546541330496,	10.0},
			{7.690416684692304,	723.0473882643314,	10.0},
			{7.707827154017987,	713.1328235799464,	10.0},
			{7.724999558885607,	703.3817033777069,	10.0},
			{7.741937199717378,	693.7657684693697,	10.0},
			{7.7586427135633595,	684.2578471313983,	10.0},
			{7.775118102381727,	674.8319260003061,	10.0},
			{7.791364762639879,	665.4632041739147,	10.0},
			{7.807383515797394,	656.128129331787,	10.0},
			{7.8231746393419215,	646.8044203839136,	10.0},
			{7.838737898062838,	637.4710772087617,	10.0},
			{7.85407257525965,	628.1083779814478,	10.0},
			{7.86917750367318,	618.6978678181388,	10.0},
			{7.884051095947764,	609.2223395669804,	10.0},
			{7.898691374436479,	599.6658068977415,	10.0},
			{7.91309600026094,	590.01347376993,	10.0},
			{7.927262301508293,	580.2516990915766,	10.0},
			{7.94118730051258,	570.3679592156077,	10.0},
			{7.95486774019603,	560.3508094341341,	10.0},
			{7.968300109426907,	550.1898436966866,	10.0},
			{7.981480667401709,	539.8756546479198,	10.0},
			{7.9944054670963824,	529.3997954938258,	10.0},
			{8.007070377782107,	518.7547416873253,	10.0},
			{8.01947110667687,	507.93385552944886,	10.0},
			{8.031603219733535,	496.93135080107146,	10.0},
			{8.043462161661427,	485.74226136635997,	10.0},
			{8.05504327522532,	474.36241157709134,	10.0},
			{8.066341819846073,	462.7883876660437,	10.0},
			{8.077352989581339,	451.0175123565355,	10.0},
			{8.088071930532239,	439.0478213487972,	10.0},
			{8.098493757748665,	426.8780427848074,	10.0},
			{8.10861357159068,	414.50757496900206,	10.0},
			{8.11842647372073,	401.9364712468208,	10.0},
			{8.12792758261925,	389.16542048343007,	10.0},
			{8.137112048750339,	376.1957327294092,	10.0},
			{8.145975069355371,	363.0293239821194,	10.0},
			{8.154511902894539,	349.6687017642883,	10.0},
			{8.162717883150988,	336.116951304069,	10.0},
			{8.17058843300391,	322.3777219757263,	10.0},
			{8.178119077853662,	308.45521304584656,	10.0},
			{8.185305458706486,	294.35415973163634,	10.0},
			{8.192143344899645,	280.0798184717879,	10.0},
			{8.198628646451581,	265.6379515673425,	10.0},
			{8.204757426016416,	251.03481097561743,	10.0},
			{8.210525910416719,	236.2771210364161,	10.0},
			{8.215930501729225,	221.37206016026738,	10.0},
			{8.220967787893198,	206.3272412762782,	10.0},
			{8.225634552820116,	191.15069140662865,	10.0},
			{8.229927785949952,	175.85082899808222,	10.0},
			{8.233844691248775,	160.43644103974927,	10.0},
			{8.237382695597944,	144.91665814199263,	10.0},
			{8.24053945654666,	129.3009284594082,	10.0},
			{8.243312869404853,	113.5989906715898,	10.0},
			{8.24570107363474,	97.82084525614236,	10.0},
			{8.24770245852552,	81.97672512639353,	10.0},
			{8.249323620252127,	66.40278432180196,	10.0},
			{8.250593145554815,	51.99975639806241,	10.0},
			{8.251553693702448,	39.344052127069325,	10.0},
			{8.252248203694279,	28.44712926538703,	10.0},
			{8.252719822793077,	19.317518286741766,	10.0},
			{8.253011845260087,	11.961240248718235,	10.0},
			{8.253167661631025,	6.3822385535926065,	10.0},
			{8.253230718765941,	2.582820246215496,	10.0},
			{8.253244490818902,	0.5641032892770436,	10.0},
			{8.253244490818902,	0.0,	10.0},
	};

}