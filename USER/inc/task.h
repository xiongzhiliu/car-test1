#ifndef TASK_H
#define TASK_H

#include "headfiles.h"

#define turnLeft 1
#define turnRight 3
#define turnBack 0
#define turnUp 2
#define stop 5
#define wrong 6
typedef struct{ 													//地图结构体
	int x;														//x坐标
	int y;														//y坐标
	uint8_t qian;											//前方是否有路
	uint8_t hou;											//后方是否有路
	uint8_t left;											//左方是否有路
	uint8_t right;										//右方是否有路 
	uint8_t qian_view;
	uint8_t hou_view;
	uint8_t left_view;
	uint8_t right_view;
	uint8_t node_num;									//岔路数量
	uint8_t no;												//编号
	struct map *next_qian;						//下一个前方的点
	struct map *next_hou;							//下一个后方的点
	struct map *next_left;						//下一个左方的点
	struct map *next_right;						//下一个右方的点
	struct map *next;									//下一个点
//	struct map *pervious;
} map;

void start_point(void);
int add_point(float get_y, float get_x, int get_left, int get_qian, int get_right, int get_hou);
int gogogo(float get_x, float get_y, int get_left, int get_qian, int get_right, int get_hou);

void Turn_right(void);
void Turn_left(void);
void Turn_back(void);
void Turn_up(void);
void  Clear_FLAG(void);

void calc_node_num(void);
int max_node_view(void);
int get_min_node_view(void);
int judge_node(struct map *node);


void task1();
#endif