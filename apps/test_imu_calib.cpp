// IMU 校准入口代码
#include <iostream>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

int main(int argc, char** argv)
{
  // 命令调用形式：./test_imu_calib test_data/xsens_acc.mat test_data/xsens_gyro.mat
  // 换句话说，argv[1] 是加计输出，argv[2] 是陀螺仪输出
  if( argc < 3 )
    return -1;

  // TriaData 定义在 `imu_tk/base.h` 中，本质是TriadData_<double>
  // 用于保存一个三元组以及一个时间戳，用来保存陀螺仪数据或者加计数据
  vector< TriadData > acc_data, gyro_data;
  
  // 1. 读取原始数据
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;  
  // 该方法定义在 imu_tk/io_utils.h 中，实现在 io_utils.cpp 中
  // 该方法有四个参数（这里是三个），前面两个分别是数据的来源以及去向
  // 后面两个分别是 timestamp 的单位以及原始数据文件中数据的分割符号（空格和逗号）
  importAsciiData( argv[1], acc_data, imu_tk::TIMESTAMP_UNIT_SEC );
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[2]<<endl;  
  importAsciiData( argv[2], gyro_data, imu_tk::TIMESTAMP_UNIT_SEC  );
  
  // 2. 校验准备
  // 该类定义在 `imu_tk/calibration.h` 中，本质是 CalibratedTriad_<double>
  CalibratedTriad init_acc_calib, init_gyro_calib;
  init_acc_calib.setBias( Vector3d(32768, 32768, 32768) );
  init_gyro_calib.setScale( Vector3d(1.0/6258.0, 1.0/6258.0, 1.0/6258.0) );
  MultiPosCalibration mp_calib;
  mp_calib.setInitStaticIntervalDuration(50.0); // 时间周期
  mp_calib.setInitAccCalibration( init_acc_calib ); // 加计数据
  mp_calib.setInitGyroCalibration( init_gyro_calib );  // 陀螺仪数据
  mp_calib.setGravityMagnitude(9.81744);  // 设置g
  mp_calib.enableVerboseOutput(true);  // 是否输出中间结果
  mp_calib.enableAccUseMeans(false);  // 
  //mp_calib.setGyroDataPeriod(0.01);

  // 3. 执行校准，输出结果
  mp_calib.calibrateAccGyro(acc_data, gyro_data );
  mp_calib.getAccCalib().save("test_imu_acc.calib");
  mp_calib.getGyroCalib().save("test_imu_gyro.calib");
  
//   for( int i = 0; i < acc_data.size(); i++)
//   {
//     cout<<acc_data[i].timestamp()<<" "
  //         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
  //         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
//   }
//   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;
  
  return 0;
}