// IMU У׼��ڴ���
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
  // ���������ʽ��./test_imu_calib test_data/xsens_acc.mat test_data/xsens_gyro.mat
  // ���仰˵��argv[1] �ǼӼ������argv[2] �����������
  if( argc < 3 )
    return -1;

  // TriaData ������ `imu_tk/base.h` �У�������TriadData_<double>
  // ���ڱ���һ����Ԫ���Լ�һ��ʱ����������������������ݻ��߼Ӽ�����
  vector< TriadData > acc_data, gyro_data;
  
  // 1. ��ȡԭʼ����
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;  
  // �÷��������� imu_tk/io_utils.h �У�ʵ���� io_utils.cpp ��
  // �÷������ĸ���������������������ǰ�������ֱ������ݵ���Դ�Լ�ȥ��
  // ���������ֱ��� timestamp �ĵ�λ�Լ�ԭʼ�����ļ������ݵķָ���ţ��ո�Ͷ��ţ�
  importAsciiData( argv[1], acc_data, imu_tk::TIMESTAMP_UNIT_SEC );
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[2]<<endl;  
  importAsciiData( argv[2], gyro_data, imu_tk::TIMESTAMP_UNIT_SEC  );
  
  // 2. У��׼��
  // ���ඨ���� `imu_tk/calibration.h` �У������� CalibratedTriad_<double>
  CalibratedTriad init_acc_calib, init_gyro_calib;
  init_acc_calib.setBias( Vector3d(32768, 32768, 32768) );
  init_gyro_calib.setScale( Vector3d(1.0/6258.0, 1.0/6258.0, 1.0/6258.0) );
  MultiPosCalibration mp_calib;
  mp_calib.setInitStaticIntervalDuration(50.0); // ʱ������
  mp_calib.setInitAccCalibration( init_acc_calib ); // �Ӽ�����
  mp_calib.setInitGyroCalibration( init_gyro_calib );  // ����������
  mp_calib.setGravityMagnitude(9.81744);  // ����g
  mp_calib.enableVerboseOutput(true);  // �Ƿ�����м���
  mp_calib.enableAccUseMeans(false);  // 
  //mp_calib.setGyroDataPeriod(0.01);

  // 3. ִ��У׼��������
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