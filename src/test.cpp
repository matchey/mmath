
// Binarionの使い方sample program

#include <ros/ros.h>
#include "mmath/binarion.h"

using std::cout;
using std::endl;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_binarion");
	cout << "calc binarion" << endl;

	double deg1 =  179; // [deg]
	double deg2 = -179; // [deg]

	if(argc > 2){ // 実行時引数で設定 (degree)
		deg1 = atof(argv[1]);
		deg2 = atof(argv[2]);
	}

	cout << "deg1 = " << deg1 << "\ndeg2 = " << deg2 << endl;

	Binarion b1(deg1*M_PI/180), b2(deg2*M_PI/180); // binarionを設定 (radian)

	cout << "b1 = " << b1 << endl; // binarion を表示 (-0.999848, 0.0174524)
	cout << "b2 = " << b2 << endl; // binarion を表示 (-0.999848, -0.0174524)
	
	cout << "b1 + b2 = " << b2 + b1 << " : " << (b1 + b2).getYaw("deg") << " [deg]" << endl;
	// 第2引数に"deg"をいれるとdegreeに変換
	cout << "b2 - b1 = " << b2 - b1 << " : " << b1.deviation(b2, "deg") << " [deg]" << endl;

	// 第2引数に"degree"とか入れなければradianで返す
	cout << "deg1からみたdeg2までの変位 : "
		 << Binarion::deviation(deg1*M_PI/180, deg2*M_PI/180)*180/M_PI << " [deg]" << endl;

	// cout
	// << Binarion::toYaw(2*Binarion::fromYaw(20, "deg") + Binarion::fromYaw(3, "deg")*3, "deg")
	// << endl;
	// cout
	// << (2*Binarion::fromYaw(20, "deg") + Binarion::fromYaw(3, "deg")*3).getYaw("degree") << endl;

	// cout << "dev(179, 179) : " << Binarion::deviation(179, -179, "deg") << endl;

	// if(argc == 3){
	// 	cout << "dev(" << argv[1] << ", " << argv[2] << ") : "
	// 		 << Binarion::deviation(atof(argv[1]), atof(argv[2]), "deg ") << endl;
	// }

	if(argc == 4){
		// yawからbinarion生成
		// slerpの引数に内分する比を設定 b1.slerp(b2, 0.3) でb1:b2を3:7に内分するBinarionを返す
		cout << "slerp(" << argv[1] << ", " << argv[2] << ", " << argv[3] << ") : "
			 << Binarion::fromYaw(atof(argv[1]), "deg")
			 	.slerp( Binarion::fromYaw(atof(argv[2]), "deg"), atof(argv[3]) ).getYaw("deg")
			 << endl;
	}

	// cout << "b(150) / -3.5 " << (Binarion::fromYaw(150, "deg") / -3.5).getYaw("deg") << endl;

	// b1とb2の真ん中 << "(0.5*b1 + (1-0.5)*b2) / (0.5 + (1-0.5)) : "
	cout << "b1とb2を 0.51 : (1-0.51) に内分 : "
		 << b1.slerp(b2, 0.51).getYaw("deg") << " [deg]" << endl;

	// b1.mean(b2, b3, ...) で平均のBinarionを返す
	cout << "平均の計算 [deg]" << endl;
	cout << b1.mean(b2, Binarion::fromYaw(177, "deg")).getYaw("deg") << endl;
	// mean(rad1, rad2, ...) で平均のradを返す
	cout << Binarion::mean("deg", -170.0, 170.0, 177.0) << endl;
	cout << Binarion::mean(-2.967, 2.967, 3.089) * 180 / M_PI << endl;
	cout << Binarion::mean(-2.967, 2.967) * 180 / M_PI << endl;
	cout << Binarion::mean(-2.967, 2.967, 3.14, -3.14) * 180 / M_PI << endl;

	return 0;
}

