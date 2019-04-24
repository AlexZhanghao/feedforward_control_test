#pragma once
#include<APS168.h>
#include <ErrorCodeDef.h>
#include <APS_define.h>
#include <type_def.h>
#include<iostream>

class feedforward {
public:
	feedforward();
	~feedforward();
	int Initial();
	bool NoError(I32 returnedvalue);
	I32 LoadAxisParam(I32 board_id);
	void SetParamZero();
	void PIDRead();
	void PIDWrite();
	void Close();


private:
	I32 ValidBoardId(I32 board_id_in_bits);

private:
	I32 board_id_;
	I32 invalid_board_id_;
	bool is_initialed_;
	I32 start_axis_id_;
	I32 total_axis_;
	F64 pid_kp_;
	F64 pid_ki_;
	F64 pid_kd_;
	F64 velocity_feedforward;
	F64 acceleration_feedforward;
	static const int shoulder_axis_id = 0;
	static const int elbow_axis_id = 1;
};
