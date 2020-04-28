// /***500Hz读取数据(bug)****/
// #include <ros/ros.h>                //必备
// #include <serial/serial.h>          //ROS已经内置了的串口包 
// #include <iostream>                 //输入输出库
// #include <sensor_msgs/Imu.h>
// //#include <std_msgs/Float64.h>
// #include <vector>
// //#include <sys/time.h>
// //#include <stdint.h>

// //创建一个serial对象
// serial::Serial ser;
// //定义imu数据结构体
// typedef struct Struct_Gyro_Data
// {
// 	int X_Rot_H;
// 	uint8_t X_Rot_L;
// 	int Y_Rot_H;
// 	uint8_t Y_Rot_L;
//     int Z_Rot_H;
//     uint8_t Z_Rot_L;
// 	int X_Acc_H;
// 	uint8_t X_Acc_L;
// 	int Y_Acc_H;
// 	uint8_t Y_Acc_L;
// 	int Z_Acc_H;
// 	uint8_t Z_Acc_L;
// 	uint8_t Calibration;
// }; 
// Struct_Gyro_Data Gyro_Data = {0x00, 0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 
//                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// typedef struct Struct_Gyro_Float
// {
//   double rotation_x;
//   double rotation_y;
//   double rotation_z;
//   double accel_x;
//   double accel_y;
//   double accel_z;
// };
// Struct_Gyro_Float gyro_data_float = {0.0,0.0,0.0,0.0,0.0,0.0};

// Struct_Gyro_Float zero_offset_result = {0.0,0.0,0.0,0.0,0.0,0.0}; 
// //imu启动指令
// uint8_t Gyro_On_Cmd[] = {0xaa,0x00,0xee,0xee,0xee,0x00,0x00,0x00,
//                          0x00,0x00,0x00,0x00,0x00,0x00,0xCA,0x00};
// uint8_t Gyro_Calibration_Cmd[] = {0xaa,0x00,0x55,0x55,0x55,0x00,0x00,0x00,
//                                   0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00};                         
// //声明函数原型
// bool Check_Sum(uint8_t input[17]);    //数据校验
// int Check_Sig(uint8_t input);           //符号位判断
// Struct_Gyro_Float Compute_Zero_Offset(std::vector<Struct_Gyro_Float> input);    //计算零偏

// int main (int argc, char** argv) 
// { 
//       //初始化节点 
//     ros::init(argc, argv, "serial_example_node"); 
//       //声明节点句柄 
//     ros::NodeHandle nh;
//     std::string utrl_serial_port1_;
//     nh.param<std::string>("utrlserial_port1", utrl_serial_port1_, "/dev/ttyUSB0");

//     try 
//     { 
//         //设置串口属性，并打开串口 
//         ser.setPort(utrl_serial_port1_); 
//         ser.setBaudrate(115200); 
//         serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
//         ser.setTimeout(to); 
//         ser.open();
//         //sleep(2);   //延迟2s        
//         size_t turn_on_mark = ser.write(Gyro_On_Cmd, 16);                    //给串口发送16进制指令，启动数据发送
//         //size_t calibration_on_mark = ser.write(Gyro_Calibration_Cmd, 16);   //给串口发送16进制指令，启动硬件数据校验
//         sleep(1);
//     } 
//     catch (serial::IOException& e) 
//     { 
//         ROS_ERROR_STREAM("Unable to open port "); 
//         return -1; 
//     } 
//       //检测串口是否已经打开，并给出提示信息 
//     if(ser.isOpen()) 
//     { 
//         ROS_INFO_STREAM("Serial Port initialized"); 
//     } 
//     else 
//     { 
//         return -1;     
//     }   
    
//     //创建topic发布器
//     ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu", 500);  
//     //ros::Rate loop_rate(250);
//     sensor_msgs::Imu imu_data;

//     //size_t len_buffer = ser.available();//获取缓冲区的字节数
//     //uint8_t buffer[38];
//     //int len_read = ser.read(buffer, 38);
//     std::vector<Struct_Gyro_Float> data_offset_500;
     
//     size_t len_buffer = ser.available();//获取缓冲区的字节数             

//     do
//     {
//         std::cout << "缓冲区有" << len_buffer << "个字节" << std::endl;
       
//         if(len_buffer > 0)
//         {
//             //std::cout << "缓冲区有" << len_buffer << "个字节" << std::endl;        
            
//             uint8_t buffer[len_buffer];
//             //sleep(0.5);
//             int len_read = ser.read(buffer, len_buffer);  
//             int num_buffer = sizeof(buffer)/sizeof(buffer[0]);  //计算buffer区数组长度
//             std::cout << "buffer数组有" << num_buffer << "个字节" << std::endl;
//             // std::cout << "单个字节长度：" <<sizeof(buffer[0]) << std::endl;
//             // uint8_t *p = buffer;
//             // std::cout << std::hex << (*(p+12) & 0xff) << std::endl;
        
//             for(uint8_t *ptr_buffer = buffer; (ptr_buffer - &buffer[0]) / sizeof(buffer[0]) < num_buffer; ++ptr_buffer)
//             {       
//                 uint8_t tmp_data[17];    //定义临时数组存放数据
//                 uint8_t *ptr_tmp = tmp_data;    
//                 struct Struct_Gyro_Data *ptr_struct = &Gyro_Data;

//                 if(*ptr_buffer == 0xaa && *(++ptr_buffer) == 0x00){
//                     uint8_t *ptr_tmp = ptr_buffer;
//                     std::cout << std::hex << ((*ptr_tmp) & 0xff) << std::endl;
//                     for(int i = 0; i < 17; i++)
//                     {
//                         ptr_tmp++;
//                         std::cout << std::hex << (*ptr_tmp & 0xff) << " " ;
                        
//                         tmp_data[i] = *ptr_tmp;                                                    
//                     }
//                     std::cout << " stop_1" << std::endl;
                
//                     // for(int i = 0; i < 17; i++)
//                     // {
//                     //     std::cout << unsigned(tmp_data[i]) << " " ;                                                  
//                     // }
//                     // std::cout << " stop_2" << std::endl;
//                     //std::cout << zero_offset_result.accel_x << std::endl;
//                     /*********开始数据校验：若数据正确，则赋值结构体*******/
//                     bool result = Check_Sum(tmp_data);
//                     std::cout << std::boolalpha << Check_Sum(tmp_data) << std::endl;
//                     if(result){
//                         Gyro_Data.X_Rot_L = tmp_data[3];
//                         Gyro_Data.X_Rot_H = Check_Sig(tmp_data[4]);
//                         Gyro_Data.Y_Rot_L = tmp_data[5];
//                         Gyro_Data.Y_Rot_H = Check_Sig(tmp_data[6]);
//                         Gyro_Data.Z_Rot_L = tmp_data[7];
//                         Gyro_Data.Z_Rot_H = Check_Sig(tmp_data[8]);
//                         Gyro_Data.X_Acc_L = tmp_data[9];
//                         Gyro_Data.X_Acc_H = Check_Sig(tmp_data[10]);
//                         Gyro_Data.Y_Acc_L = tmp_data[11];
//                         Gyro_Data.Y_Acc_H = Check_Sig(tmp_data[12]);
//                         Gyro_Data.Z_Acc_L = tmp_data[13];
//                         Gyro_Data.Z_Acc_H = Check_Sig(tmp_data[14]);
//                         //         /****发imu类型topic****/
//                         sensor_msgs::Imu imu_data;
//                         imu_data.header.stamp = ros::Time::now();
//                         imu_data.header.frame_id = "imu_frame";
//                         //线加速度---并将mg转化为m/s2
//                         imu_data.linear_acceleration.x = (double((Gyro_Data.X_Acc_H) * 256 + Gyro_Data.X_Acc_L)) * 9.7925 / 1000; 
//                         imu_data.linear_acceleration.y = (double((Gyro_Data.Y_Acc_H) * 256 + Gyro_Data.Y_Acc_L)) * 9.7925 / 1000;
//                         imu_data.linear_acceleration.z = (double((Gyro_Data.Z_Acc_H) * 256 + Gyro_Data.Z_Acc_L)) * 9.7925 / 1000;
//                         //旋转角速度--单位转化成“度/s”
//                         imu_data.angular_velocity.x = (double((Gyro_Data.X_Rot_H) * 256 + Gyro_Data.X_Rot_L)) / (32 * 131); 
//                         imu_data.angular_velocity.y = (double((Gyro_Data.Y_Rot_H) * 256 + Gyro_Data.Y_Rot_L)) / (32 * 131); 
//                         imu_data.angular_velocity.z = (double((Gyro_Data.Z_Rot_H) * 256 + Gyro_Data.Z_Rot_L)) / (32 * 131);
                        
//                         IMU_pub.publish(imu_data);
            
//                         ros::spinOnce();

//                         //continue;
//                     }
                                
//                 }  //若满足0xaa 0x00 则输出

//             }
//             /**结构体赋值**/
//             gyro_data_float.accel_x = (double((Gyro_Data.X_Acc_H) * 256 + Gyro_Data.X_Acc_L)) * 9.7925 / 1000;
//             gyro_data_float.accel_y = (double((Gyro_Data.Y_Acc_H) * 256 + Gyro_Data.Y_Acc_L)) * 9.7925 / 1000;
//             gyro_data_float.accel_z = (double((Gyro_Data.Z_Acc_H) * 256 + Gyro_Data.Z_Acc_L)) * 9.7925 / 1000;
//             gyro_data_float.rotation_x = (double((Gyro_Data.X_Rot_H) * 256 + Gyro_Data.X_Rot_L)) / (32 * 131); 
//             gyro_data_float.rotation_y = (double((Gyro_Data.Y_Rot_H) * 256 + Gyro_Data.Y_Rot_L)) / (32 * 131); 
//             gyro_data_float.rotation_z = (double((Gyro_Data.Z_Rot_H) * 256 + Gyro_Data.Z_Rot_L)) / (32 * 131);
//             /**数据取零偏：计算前500个数据的平均值***/
//             // if(data_offset_500.size() < 500){
//             //         data_offset_500.push_back(gyro_data_float);
//             //         std::cout << int(data_offset_500.size()) << std::endl;
//             //     };

//             // if(data_offset_500.size() == 500){
//             //     zero_offset_result = Compute_Zero_Offset(data_offset_500);
//             // }
//             // else{
//             //     zero_offset_result = {0,0,0,0,0,0};
//             // }
                
//         }
//     }
//     while(len_buffer > 19);
//     return 0;

// }
// //数据校验
// bool Check_Sum(uint8_t input[17])      
// {
//     bool check = false;
//     uint8_t sum = 0x00;
//     for (int i = 0; i < 16; i++)
//     {
//       sum += input[i];
//     }
//     if (sum == input[16])
//     {
//         check = true;/* code */
//     }
//     else
//     {
//         check = false;
//     }
//     return check;
// }
// //符号位判断
// int Check_Sig(uint8_t input)
// {
//     if (input > 128)
//     {
//         return input - 256;
//     }
//     else
//     {
//        return input;
//     }    
// }
// //零偏值计算
// Struct_Gyro_Float Compute_Zero_Offset(std::vector<Struct_Gyro_Float> input)
// {
//     Struct_Gyro_Float result = {0,0,0,0,0,0};
//     double sum_acc_x, sum_acc_y, sum_acc_z, sum_rot_x, sum_rot_y, sum_rot_z = 0;

//     for (int i = 0; i < input.size(); i++)
//     {
//         sum_acc_x += input[i].accel_x;
//         sum_acc_y += input[i].accel_y;
//         sum_acc_z += input[i].accel_z;
//         sum_rot_x += input[i].rotation_x;
//         sum_rot_y += input[i].rotation_y;
//         sum_rot_z += input[i].rotation_z;
//     }
//     result.accel_x = sum_acc_x / input.size();
//     result.accel_y = sum_acc_y / input.size();
//     result.accel_z = sum_acc_z / input.size();
//     result.rotation_x = sum_rot_x / input.size();
//     result.rotation_y = sum_rot_y / input.size();
//     result.rotation_z = sum_rot_z / input.size();

//     return result;
// }

/***偶然性****/
// #include <ros/ros.h>                //必备
// #include <serial/serial.h>          //ROS已经内置了的串口包 
// #include <iostream>                 //输入输出库
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Float32.h>
// #include <sys/time.h>

// //创建一个serial 类
// serial::Serial ser;
// //定义imu数据结构体
// typedef struct
// {
// 	uint8_t X_Rot_H;
// 	uint8_t X_Rot_L;
// 	uint8_t Y_Rot_H;
// 	uint8_t Y_Rot_L;
//     uint8_t Z_Rot_H;
//     uint8_t Z_Rot_L;
// 	uint8_t X_Acc_H;
// 	uint8_t X_Acc_L;
// 	uint8_t Y_Acc_H;
// 	uint8_t Y_Acc_L;
// 	uint8_t Z_Acc_H;
// 	uint8_t Z_Acc_L;
// 	uint8_t Calibration;
// } Struct_Gyro_Data;
// Struct_Gyro_Data Gyro_Data = {0,0,0,0,0,0,0,0,0,0,0};
// //imu启动指令
// uint8_t Gyro_On_Cmd[] = {0xaa,0x00,0xee,0xee,0xee,0x00,0x00,0x00,
//                          0x00,0x00,0x00,0x00,0x00,0x00,0xCA,0x00};
// // void drive_serial_imu(const std_msgs::Hex)
// // {

// // }

// int main (int argc, char** argv) 
// { 
//       //初始化节点 
//     ros::init(argc, argv, "serial_example_node"); 
//       //声明节点句柄 
//     ros::NodeHandle nh; 

//     try 
//     { 
//         //设置串口属性，并打开串口 
//         ser.setPort("/dev/ttyUSB0"); 
//         ser.setBaudrate(115200); 
//         serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
//         ser.setTimeout(to); 
//         ser.open();
//         //sleep(2);   //延迟2s        
//         size_t turn_on_mark = ser.write(Gyro_On_Cmd, 16);   //给串口发送16进制指令，启动数据发送
//         //sleep(1);
//     } 
//     catch (serial::IOException& e) 
//     { 
//         ROS_ERROR_STREAM("Unable to open port "); 
//         return -1; 
//     } 
//       //检测串口是否已经打开，并给出提示信息 
//     if(ser.isOpen()) 
//     { 
//         ROS_INFO_STREAM("Serial Port initialized"); 
//     } 
//     else 
//     { 
//         return -1; 
//     }
//     //创建topic发布器
//     ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU_data", 500);  
//     //指定循环的频率 
//     ros::Rate loop_rate(1000);
//     sensor_msgs::Imu imu_data; 
//     while(ros::ok()) 
//     { 
//         //获取缓冲区的字节数
//         size_t len_buffer = ser.available();
//         if(len_buffer > 0)
//         {
//             std::cout << "缓冲区有" << len_buffer << "个字节" << std::endl;
        
//             uint8_t buffer[19];
//             //读取数据
//             int len_read = ser.read(buffer, 19);
//             //std::cout << len_read << std::endl;
//           if((buffer[0] == 0xaa) && (buffer[1] == 0x00))
//           {
              
//               for(int i = 0; i < 19; i++)
//               {
//                 //16进制输出
//                 std::cout << std::hex << (buffer[i] & 0xff) << " " ;                
//               }
// 					      Gyro_Data.X_Rot_H = buffer[6];
// 					      Gyro_Data.X_Rot_L = buffer[5];
// 					      Gyro_Data.Y_Rot_H = buffer[8];
//                           Gyro_Data.Y_Rot_L = buffer[7];
//                           Gyro_Data.Z_Rot_H = buffer[3];
//                           Gyro_Data.Z_Rot_L = buffer[9];
// 					      Gyro_Data.X_Acc_H = buffer[6];
// 					      Gyro_Data.X_Acc_L = buffer[5];
// 					      Gyro_Data.Y_Acc_H = buffer[8];
// 					      Gyro_Data.Y_Acc_L = buffer[7];
// 					      Gyro_Data.Z_Acc_H = buffer[10];
// 					      Gyro_Data.Z_Acc_L = buffer[9];
//           }
//           // std_msgs::Float32 Yaw;
//           // Yaw.data = (float)((Gyro_Data.X_Acc_H * 256 + Gyro_Data.X_Acc_L));
//           // std::cout << Yaw << std::endl;

//         /****发imu类型topic****/
//         sensor_msgs::Imu imu_data;
//         imu_data.header.stamp = ros::Time::now();
//         imu_data.header.frame_id = "imu_frame";
//         //线加速度
//         imu_data.linear_acceleration.x = 0.01; 
//         imu_data.linear_acceleration.y = 0.02;
//         imu_data.linear_acceleration.z = 0.03;
//         //旋转角速度
//         imu_data.angular_velocity.x = 0.05; 
//         imu_data.angular_velocity.y = 0.06; 
//         imu_data.angular_velocity.z = 0.07;
        
//         IMU_pub.publish(imu_data);
        
//         ros::spinOnce();
//         loop_rate.sleep();
//       }
//     }
// }



/***250Hz读取数据****/
#include <ros/ros.h>                //必备
#include <serial/serial.h>          //ROS已经内置了的串口包 
#include <iostream>                 //输入输出库
#include <sensor_msgs/Imu.h>
//#include <std_msgs/Float64.h>
#include <vector>
#include <numeric>
//#include <sys/time.h>
//#include <stdint.h>

//创建一个serial对象
serial::Serial ser;
//定义imu数据结构体
typedef struct Struct_Gyro_Data
{
	int X_Rot_H;
	uint8_t X_Rot_L;
	int Y_Rot_H;
	uint8_t Y_Rot_L;
    int Z_Rot_H;
    uint8_t Z_Rot_L;
	int X_Acc_H;
	uint8_t X_Acc_L;
	int Y_Acc_H;
	uint8_t Y_Acc_L;
	int Z_Acc_H;
	uint8_t Z_Acc_L;
	uint8_t Calibration;
}; 
Struct_Gyro_Data Gyro_Data = {0x00, 0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
typedef struct Struct_Gyro_Float
{
  double rotation_x;
  double rotation_y;
  double rotation_z;
  double accel_x;
  double accel_y;
  double accel_z;
};
struct Offset
{
    double x;
    double y;
    double z;/* data */
};
Struct_Gyro_Float gyro_data_float = {0.0,0.0,0.0,0.0,0.0,0.0};

Offset zero_offset_result = {0.0,0.0,0.0};
double z_offset_average = 0.0; 
//imu启动指令
uint8_t Gyro_On_Cmd[] = {0xaa,0x00,0xee,0xee,0xee,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0xCA,0x00};
uint8_t Gyro_Calibration_Cmd[] = {0xaa,0x00,0x55,0x55,0x55,0x00,0x00,0x00,
                                  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00};                         
//声明函数原型
bool Check_Sum(uint8_t input[17]);    //数据校验
int Check_Sig(uint8_t input);           //符号位判断
Offset Compute_Zero_Offset(std::vector<Struct_Gyro_Float> input);    //计算零偏

int main (int argc, char** argv) 
{ 
      //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
      //声明节点句柄 
    ros::NodeHandle nh;
    std::string utrl_serial_port1_;
    nh.param<std::string>("utrlserial_port1", utrl_serial_port1_, "/dev/ttyUSB0");

    try 
    { 
        //设置串口属性，并打开串口 
        ser.setPort(utrl_serial_port1_); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open();
        //sleep(2);   //延迟2s        
        size_t turn_on_mark = ser.write(Gyro_On_Cmd, 16);                    //给串口发送16进制指令，启动数据发送
        //size_t calibration_on_mark = ser.write(Gyro_Calibration_Cmd, 16);   //给串口发送16进制指令，启动硬件数据校验
        //sleep(1);
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
      //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    
    }   
    
    //创建topic发布器
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);  
    ros::Rate loop_rate(500);
    sensor_msgs::Imu imu_data;

    //size_t len_buffer = ser.available();//获取缓冲区的字节数
    //uint8_t buffer[38];
    //int len_read = ser.read(buffer, 38);
    std::vector<Struct_Gyro_Float> data_offset_500;

        while(ros::ok()) 
        { 
            size_t len_buffer = ser.available();//获取缓冲区的字节数             
            //std::cout << "缓冲区有" << len_buffer << "个字节" << std::endl;

            if(len_buffer > 0)
            {
                //std::cout << "缓冲区有" << len_buffer << "个字节" << std::endl;        
                
                uint8_t buffer[38];
                int len_read = ser.read(buffer, 38);  
                //std::cout << len_read << std::endl;
                int num_buffer = sizeof(buffer)/sizeof(buffer[0]);  //计算buffer区数组长度
                //std::cout << "buffer数组有" << num_buffer << "个变量" << std::endl;
                // std::cout << "单个字节长度：" <<sizeof(buffer[0]) << std::endl;
                // uint8_t *p = buffer;
                // std::cout << std::hex << (*(p+12) & 0xff) << std::endl;
            
            for(uint8_t *ptr_buffer = buffer; (ptr_buffer - &buffer[0]) / sizeof(buffer[0]) < num_buffer; ++ptr_buffer)
            {       
                uint8_t tmp_data[17];    //定义临时数组存放数据
                uint8_t *ptr_tmp = tmp_data;    
                struct Struct_Gyro_Data *ptr_struct = &Gyro_Data;

                if(*ptr_buffer == 0xaa && *(++ptr_buffer) == 0x00){
                    // uint8_t *ptr_tmp = ptr_buffer;
                    // std::cout << std::hex << ((*(++ptr_tmp)) & 0xff) << std::endl;
                    for(int i = 0; i < 17; i++)
                    {
                            ++ptr_buffer;
                            std::cout << std::hex << (*ptr_buffer & 0xff) << " " ;
                            tmp_data[i] = *ptr_buffer;                                                    
                    }
                    std::cout << " stop_1" << std::endl;
                   
                    for(int i = 0; i < 17; i++)
                    {
                        std::cout << unsigned(tmp_data[i]) << " " ;                                                  
                    }
                    std::cout << " stop_2" << std::endl;
                    //std::cout << zero_offset_result.accel_x << std::endl;
                    /*********开始数据校验：若数据正确，则赋值结构体*******/
                    bool result = Check_Sum(tmp_data);
                    std::cout << std::boolalpha << Check_Sum(tmp_data) << std::endl;
                    if(result){
                        Gyro_Data.X_Rot_L = tmp_data[3];
                        Gyro_Data.X_Rot_H = Check_Sig(tmp_data[4]);
                        Gyro_Data.Y_Rot_L = tmp_data[5];
                        Gyro_Data.Y_Rot_H = Check_Sig(tmp_data[6]);
                        Gyro_Data.Z_Rot_L = tmp_data[7];
                        Gyro_Data.Z_Rot_H = Check_Sig(tmp_data[8]);
                        Gyro_Data.X_Acc_L = tmp_data[9];
                        Gyro_Data.X_Acc_H = Check_Sig(tmp_data[10]);
                        Gyro_Data.Y_Acc_L = tmp_data[11];
                        Gyro_Data.Y_Acc_H = Check_Sig(tmp_data[12]);
                        Gyro_Data.Z_Acc_L = tmp_data[13];
                        Gyro_Data.Z_Acc_H = Check_Sig(tmp_data[14]);
                    }
                    break;               
                }  //若满足0xaa 0x00 则输出
                else
                {
                    continue;
                }  //若不满足0xaa 0x00则指针移动一个字节 
            }
            /**结构体赋值**/
            gyro_data_float.accel_x = (double((Gyro_Data.X_Acc_H) * 256 + Gyro_Data.X_Acc_L)) * 9.7925 / 1000;
            gyro_data_float.accel_y = (double((Gyro_Data.Y_Acc_H) * 256 + Gyro_Data.Y_Acc_L)) * 9.7925 / 1000;
            gyro_data_float.accel_z = (double((Gyro_Data.Z_Acc_H) * 256 + Gyro_Data.Z_Acc_L)) * 9.7925 / 1000;
            gyro_data_float.rotation_x = (double((Gyro_Data.X_Rot_H) * 256 + Gyro_Data.X_Rot_L)) / (32 * 131); 
            gyro_data_float.rotation_y = (double((Gyro_Data.Y_Rot_H) * 256 + Gyro_Data.Y_Rot_L)) / (32 * 131); 
            gyro_data_float.rotation_z = (double((Gyro_Data.Z_Rot_H) * 256 + Gyro_Data.Z_Rot_L)) / (32 * 131);
            /**数据取零偏：计算前500个数据的平均值***/
            if(data_offset_500.size() < 500){
                    data_offset_500.push_back(gyro_data_float);
                    std::cout << int(data_offset_500.size()) << std::endl;
                };

            if(data_offset_500.size() == 500){
                zero_offset_result = Compute_Zero_Offset(data_offset_500);
                
            }

            /****发imu类型topic****/
            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "imu_frame";
            //线加速度---并将mg转化为m/s2
            imu_data.linear_acceleration.x = (double((Gyro_Data.X_Acc_H) * 256 + Gyro_Data.X_Acc_L)) * 9.7925 / 1000 - zero_offset_result.x; 
            imu_data.linear_acceleration.y = (double((Gyro_Data.Y_Acc_H) * 256 + Gyro_Data.Y_Acc_L)) * 9.7925 / 1000 - zero_offset_result.y;
            imu_data.linear_acceleration.z = (double((Gyro_Data.Z_Acc_H) * 256 + Gyro_Data.Z_Acc_L)) * 9.7925 / 1000 - zero_offset_result.z;
            //旋转角速度--单位转化成“度/s”
            imu_data.angular_velocity.x = -(double((Gyro_Data.X_Rot_H) * 256 + Gyro_Data.X_Rot_L)) / (32 * 131); 
            imu_data.angular_velocity.y = -(double((Gyro_Data.Y_Rot_H) * 256 + Gyro_Data.Y_Rot_L)) / (32 * 131); 
            imu_data.angular_velocity.z = (double((Gyro_Data.Z_Rot_H) * 256 + Gyro_Data.Z_Rot_L)) / (32 * 131);

            IMU_pub.publish(imu_data);
              
            ros::spinOnce();
                    
            }
        loop_rate.sleep();
        }
}
//数据校验
bool Check_Sum(uint8_t input[17])      
{
    bool check = false;
    uint8_t sum = 0x00;
    for (int i = 0; i < 16; i++)
    {
      sum += input[i];
    }
    if (sum == input[16])
    {
        check = true;/* code */
    }
    else
    {
        check = false;
    }
    return check;
}
//符号位判断
int Check_Sig(uint8_t input)
{
    if (input > 128)
    {
        return input - 256;
    }
    else
    {
       return input;
    }    
}
//计算x，y, z方向零偏值
Offset Compute_Zero_Offset(std::vector<Struct_Gyro_Float> input)
{
    Struct_Gyro_Float result = {0,0,0,0,0,0};

    Offset offset = {0.0, 0.0, 0.0};
    
    double z_offset_average;
    double sum_acc_x, sum_acc_y, sum_acc_z, sum_rot_x, sum_rot_y, sum_rot_z = 0;

    for (int i = 0; i < input.size(); i++)
    {
        sum_acc_x += input[i].accel_x;
        sum_acc_y += input[i].accel_y;
        sum_acc_z += input[i].accel_z;
        sum_rot_x += input[i].rotation_x;
        sum_rot_y += input[i].rotation_y;
        sum_rot_z += input[i].rotation_z;
    }
    result.accel_x = sum_acc_x / input.size();
    result.accel_y = sum_acc_y / input.size();
    result.accel_z = sum_acc_z / input.size();
    result.rotation_x = sum_rot_x / input.size();
    result.rotation_y = sum_rot_y / input.size();
    result.rotation_z = sum_rot_z / input.size();

    std::vector<double> z_offset;
    for (int i = 0; i < input.size(); i++)
    {
        z_offset.push_back(input[i].accel_z - result.accel_z);/* code */
        z_offset_average = accumulate(z_offset.begin(), z_offset.begin(), 0);
    }
    offset.x = result.accel_x;
    offset.y = result.accel_y;
    offset.z = z_offset_average;
    
    return offset;
}
