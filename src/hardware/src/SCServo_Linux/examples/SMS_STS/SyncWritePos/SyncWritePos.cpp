/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

u8 ID[3] = {1, 2, 3};
s16 Position[3];
// u16 Speed[3] = {2400, 2400, 2400};
// u8 ACC[3] = {50, 50, 50};
u16 Speed[3] = {1000, 1000, 1000};
u8 ACC[3] = {20, 20, 20};

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(1000000, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
	while(1){
		Position[0] = 1420;
		Position[1] = 680;
		Position[2] = 110;
		sm_st.SyncWritePosEx(ID, sizeof(ID), Position, Speed, ACC);//舵机(ID1/ID2)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
		std::cout<<"pos = "<< "min"<<std::endl;
		usleep(2187*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
  
		Position[0] = 3710;
		Position[1] = 3520;
		Position[2] = 2000;
		sm_st.SyncWritePosEx(ID, sizeof(ID), Position, Speed, ACC);//舵机(ID1/ID2)以最高速度V=2400(步/秒)，加速度A=50(50*100步/秒^2)，运行至P0=0位置
		std::cout<<"pos = max "<<std::endl;
		usleep(2187*1000);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	}
	sm_st.end();
	return 1;
}

