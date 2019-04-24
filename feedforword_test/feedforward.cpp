#include"feedforward.h"

using namespace std;

feedforward::feedforward() {
	 board_id_ = 0;
	 invalid_board_id_ = 0;
	 is_initialed_ = false;
	 start_axis_id_ = 0;
	 total_axis_ = 0;
	 pid_kp_ = 0;
	 pid_ki_ = 0;
	 pid_kd_ = 0;
	 velocity_feedforward = 0;
	 acceleration_feedforward = 0;
}

feedforward::~feedforward() {
	Close();
}

int feedforward::Initial() {
	I32 board_id_in_bits = 0;
	I32 mode = 0;

	if (NoError(APS_initial(&board_id_in_bits, mode))) {
		board_id_ = ValidBoardId(board_id_in_bits);
		if (board_id_ == invalid_board_id_) {
			is_initialed_ = false;
			return -1;
		}
		LoadAxisParam(board_id_);
		SetParamZero();
		APS_load_parameter_from_flash(board_id_);
		is_initialed_ = true;
	}
	return 0;
}

bool feedforward::NoError(I32 returnedvalue) {
	return returnedvalue == ERR_NoError;
}

I32 feedforward::ValidBoardId(I32 board_id_in_bits) {
	for (I32 id = 0; id < 16; id++) {
		int t = (board_id_in_bits >> id) & 1;
		if (t == 1) {
			I32 card_name = 0;
			APS_get_card_name(id, &card_name);
			if (card_name == DEVICE_NAME_PCI_8258 || card_name == DEVICE_NAME_AMP_82548) {
				return id;
			}
		}
	}
	return invalid_board_id_;
}

I32 feedforward::LoadAxisParam(I32 board_id) {
	return APS_get_first_axisId(board_id, &start_axis_id_, &total_axis_);
}

void feedforward::SetParamZero() {
	for (I32 axis_id = 0; axis_id < total_axis_; axis_id++) {
		APS_set_command_f(axis_id, 0.0);//设置命令位置为0
		APS_set_position_f(axis_id, 0.0);//设置反馈位置为0
	}
}

void feedforward::PIDRead() {
	I32 rp = 0, ri = 0, rd = 0, rkvff = 0, rkaff = 0;
	for (int id = shoulder_axis_id; id <= elbow_axis_id; ++id) {
		I32 rp=APS_get_axis_param_f(id, PRA_KP_GAIN, &pid_kp_);//获取PID中的比例参数
		I32 ri=APS_get_axis_param_f(id, PRA_KI_GAIN, &pid_ki_);//获取PID中的积分参数
		I32 rd=APS_get_axis_param_f(id, PRA_KD_GAIN, &pid_kd_);//获取PID中的微分参数
		I32 rkvff=APS_get_axis_param_f(id, PRA_KFF_GAIN, &velocity_feedforward);//获取速度前馈
		I32 rkaff=APS_get_axis_param_f(id, PRA_KAFF_GAIN, &acceleration_feedforward);//获取加速度前馈
	}
	if (rp == 0 && ri == 0 && rd == 0 && rkvff == 0 && rkaff == 0) {
		cout << "P:" << pid_kp_ << "\n" << "I:" << pid_ki_ << "\n" << "D:" << pid_kd_ << endl;
		cout << "KVFF:" << velocity_feedforward << "\n" << "KAFF:" << acceleration_feedforward << endl;
	}
	else {
		cout << "PID参数读取失败！" << endl;
		cout << "rp:" << rp << "  " << "ri:" << ri << "  " << "rd:" << rd << endl;
		cout << "rkvff:" << rkvff << "  " << "rkaff:" << rkaff << endl;
	}
}

void feedforward::PIDWrite() {
	F64 p = 0, i = 0, d = 0;
	F64 kvff = 0, kaff = 0;
	I32 rp = 0, ri = 0, rd = 0, rkvff = 0, rkaff = 0;
	for (int id = shoulder_axis_id; id <= elbow_axis_id; ++id) {
		rp=APS_set_axis_param_f(id, PRA_KP_GAIN, p);//写入PID中的比例参数
		ri=APS_set_axis_param_f(id, PRA_KI_GAIN, i);//写入PID中的积分参数
		rd=APS_set_axis_param_f(id, PRA_KD_GAIN, d);//写入PID中的微分参数
		rkvff=APS_set_axis_param_f(id, PRA_KFF_GAIN, kvff);//写入速度前馈
		rkaff=APS_set_axis_param_f(id, PRA_KAFF_GAIN, kaff);//写入加速度前馈
	}
	if (rp == 0 && ri == 0 && rd == 0 && rkvff == 0 && rkaff == 0) {
		
	}
	else {
		cout << "PID参数写入失败！" << endl;
		cout << "rp:" << rp << "  " << "ri:" << ri << "  " << "rd:" << rd << endl;
		cout << "rkvff:" << rkvff << "  " << "rkaff:" << rkaff << endl;
	}
}

void feedforward::Close() {
	APS_close();
}