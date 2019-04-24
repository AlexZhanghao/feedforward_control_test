#include"feedforward.h"

int main() {
	feedforward a;
	a.Initial();
	a.PIDRead();
	a.PIDWrite();
	system("pause");
	return 0;
}