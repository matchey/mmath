
// Binarionの使い方sample program

#include <ros/ros.h>
#include "mmath/binarion.h"

using namespace std;

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

	Binarion b1(deg1*M_PI/180), b2(deg2*M_PI/180); // binarionを設定 (radian)

	cout << "b1 : " << b1 << endl; // binarion を表示 (-0.999848, 0.0174524)
	cout << "b2 : " << b2 << endl; // binarion を表示 (-0.999848, -0.0174524)
	// cout << "b1 + b2 : " << b2 + b1 << " : " << (b1 + b2).getYaw("deg") << endl;
	
	// 第2引数に"deg"をいれるとdegreeに変換
	cout << "b2 - b1 : " << b2 - b1 << " : " << b1.deviation(b2, "deg") << endl;

	// 第2引数に"degree"とか入れなければradianで返す
	cout << "deg2 - deg1 : " << Binarion::deviation(deg1*M_PI/180, deg2*M_PI/180)*180/M_PI<< endl;

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

	cout << b1.slerp(b2, 0.5).getYaw("deg") << endl;

	// mean(b1, b2, ...) で平均のBinarionを返す
	cout << b1.mean(b2, Binarion::fromYaw(177, "deg")).getYaw("deg") << endl;
	cout << Binarion::mean("deg", -170.0, 170.0, 177.0) << endl;
	cout << Binarion::mean(-2.967, 2.967, 3.089) * 180 / M_PI << endl;
	cout << Binarion::mean(-2.967, 2.967) * 180 / M_PI << endl;
	cout << Binarion::mean(-2.967, 2.967, 3.14, -3.14) * 180 / M_PI << endl;

	return 0;
}

