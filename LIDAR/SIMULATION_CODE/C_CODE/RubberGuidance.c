#pragma once
#ifndef RUBBER_C
#define RUBBER_C
extern double buf_AFdeltaf[];
#include "RubberGuidance.h"

void FieldBuffer()
{
	for (int m = 0; m < MAX_LIDAR_NUM; m++)
	{
		OFF.angle[m] = (double)((0.25 * m - 95) * PI / 180); // 760개 포인트에 대해 해상도 0.25 => 160도 => -80을 해줘서 범위를 -80 ~ 80으로 바꿔준다.
		SFF.angle[m] = (double)((0.25 * m - 95) * PI / 180); // 760개 포인트에 대해 해상도 0.25 => 160도 => -80을 해줘서 범위를 -80 ~ 80으로 바꿔준다.
		IFF.angle[m] = (double)((0.25 * m - 95) * PI / 180); // 760개 포인트에 대해 해상도 0.25 => 160도 => -80을 해줘서 범위를 -80 ~ 80으로 바꿔준다.
	}

	for (int m = 0; m < MAX_LIDAR_NUM; m++)
	{
		CLUSTER.angle[m] = (double)((0.25 * m - 95) * PI / 180);
	}
}


void LidarDataRead()
{
	for (int i = 0; i < MAX_LIDAR_NUM; i++)
	{
		//*********************************************************//
		 // NULL 값 채우기
		if (i == 0)
		{
			if (Lidar_dis[i] == 0.0)   // 첫번째 인덱스 값이 0일 경우에는 1로 만들어주기
			{
				Lidar_dis[i] = 1.0;
			}
		}
		else
		{
			if (Lidar_dis[i] == 0.0)    // 값이 안들오면 이전 값 유지
			{
				Lidar_dis[i] = Lidar_dis[i - 1];
			}
		}

		//*********************************************************//
		// Normalization
		if (Lidar_dis[i] >= RangeMAX)   // 받아오는 거리값 range보다 크면 1로 normalize..  500당 1m 임
		{
			buf_dis[i] = 1;
		}
		else
		{
			buf_dis[i] = (Lidar_dis[i]) / (RangeMAX);
		}
		buf_xCoor[i] = buf_dis[i] * cos(CLUSTER.angle[i]);// +90 * DEG2RAD);
		buf_yCoor[i] = buf_dis[i] * sin(CLUSTER.angle[i]);// + 90 * DEG2RAD);
	}
	//*********************************************************//
	// lidar obstacle field expansion
}

void GenerateOFF(void)
{
	double  tmp = 0;
	double  r_tmp = 0;
	double  r_i = 0;
	double  theta_i = 0;
	double  theta = 0;
	double  D = 0;
	int16_t index;

	// 데이터 전처리
	LidarDataRead();
	memcpy(OFF.dis, buf_dis, sizeof(buf_dis));                ///////////확인

	for (int i = 0; i < MAX_LIDAR_NUM; i++)
	{
		r_i = buf_dis[i];
		theta_i = OFF.angle[i]; // radian
		k_local = K_LOCAL;
		tmp = K_LOCAL / r_i;

		if (tmp > 1.0)
		{
			tmp = 1.0;
			k_local = r_i;
		}

		theta = floor((-asin(tmp) * RAD2DEG + theta_i * RAD2DEG) * 4 + 0.5) / 4;  // 반올림

		while (theta < (asin(tmp) * RAD2DEG + theta_i * RAD2DEG))
		{
			index = (int16_t)(theta * 4);
			index = index + 380;
			if (index >= 0 && index < MAX_LIDAR_NUM)
			{
				D = r_i * r_i * pow(cos(OFF.angle[index] - theta_i), 2) - (r_i * r_i - k_local * k_local);

				if (D < 0.0)
					D = 0.0;

				r_tmp = r_i * cos(OFF.angle[index] - theta_i) - sqrt(D);

				if (r_tmp < 0.0)
					r_tmp = 0.0;

				if (r_tmp <= OFF.dis[index])
					OFF.dis[index] = r_tmp;
			}
			theta = theta + 0.25;
		}
	}
	lsr_MemWrite();
}

void GenerateSFF(void)
{
	sigSFF = 0.4;
	for (int i = 0; i < MAX_LIDAR_NUM; i++)
	{
		SFF.dis[i] = exp(-1.0 * (SFF.angle[i] - delta_f * DEG2RAD) * (SFF.angle[i] - delta_f * DEG2RAD) / (2.0 * sigSFF * sigSFF));
	}
}


void GenerateIFF(void)
{
	weightIFF = 0.5;
	int cnt = 0;

	for (int i = 0; i < MAX_LIDAR_NUM; i++)
	{
		IFF.dis[i] = weightIFF * SFF.dis[i] + (1 - weightIFF) * OFF.dis[i];

		if (IFF.dis[i] > IFF.dis[cnt])
			cnt = i;
	}

	if (IFF.angle[cnt] > MAX_ANGLE_POSITIVE)
	{
		max_angle = MAX_ANGLE_POSITIVE;
	}
	else if (IFF.angle[cnt] < MAX_ANGLE_NEGATIVE)
	{
		max_angle = MAX_ANGLE_NEGATIVE;
	}
	else
	{
		max_angle = IFF.angle[cnt];
	}
	delta_f = max_angle * RAD2DEG;
	//CMD_delta_f();
}

void Lidar_Clustering()
{
	LidarDataRead();
	memcpy(CLUSTER.dis, buf_dis, sizeof(buf_dis));
	memcpy(CLUSTER.xCoor, buf_xCoor, sizeof(buf_xCoor));
	memcpy(CLUSTER.yCoor, buf_yCoor, sizeof(buf_yCoor));

	//index = index + 360;

	//extern double dist_read[];

	for (int k = 20; k < CLUSTER_NUM + 20; k++)
	{
		disCoor[k - 20] = buf_dis[k];   //CLUSTER.dis  [k];   //
		xxCoor[k - 20] = buf_xCoor[k]; //CLUSTER.xCoor[k];   //
		yyCoor[k - 20] = buf_yCoor[k];//CLUSTER.yCoor[k];   //
	}

	dbdb(xxCoor, yyCoor);
	//printf("c: %d\n", C);
	BIND_CLUSTER();
	main_data();

}

void dbdb(double* x, double* y) {
	C = 0;
	//double(*D)[CLUSTER_NUM] = mypdist2(x, y);  // 2차원 배열 리턴

	for (int i = 0; i < CLUSTER_NUM; i++) {
		for (int j = 0; j < CLUSTER_NUM; j++) {
			value = sqrt((x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]));
			D[i][j] = value;
		}
	}
	//if (first_count == 53)
	//{

		//printf("D2 = [\n");
		//for (size_t asd = 0; asd < 721; asd++)
		//{
		//	printf("%f\n", D[286][asd]);
		//}
		//printf("];\n");
	//}
	for (size_t i = 0; i < CLUSTER_NUM; i++) // 초기화
	{
		visited[i] = 0.0;
		Neighbors[i] = 0.0;
		isnoise[i] = 0.0;
		x_in_find[i] = 0.0;
		IDX[i] = 0;
	}

	for (size_t i = 0; i < CLUSTER_NUM; i++)
	{
		memset(super_out_find, 0, count_Neighbors * sizeof(super_out_find[0]));
		if (visited[i] == 0.0)
		{
			visited[i] = 1.0;
			memset(x_in_find, 0, CLUSTER_NUM * sizeof(x_in_find[0]));
			for (size_t m = 0; m < CLUSTER_NUM; m++)
			{
				if (D[i][m] <= boundary)
				{
					x_in_find[m] = 1.0;
				}
			}
			count_in_find = 0;                           ///초기화
			memset(out_find, 0, CLUSTER_NUM * sizeof(out_find[0]));
			for (size_t m = 0; m < CLUSTER_NUM; m++)
			{
				if (x_in_find[m] != 0.0)
				{
					out_find[count_in_find] = m + 1;
					count_in_find = count_in_find + 1;
				}
			}
			count_Neighbors = 0;
			for (size_t m = 0; m < CLUSTER_NUM; m++)
			{
				if (out_find[m] != 0)
				{
					count_Neighbors = count_Neighbors + 1;
				}
			}

			if (count_Neighbors < minPoint)
			{
				isnoise[i] = 1.0;
			}
			else
			{
				C = C + 1;
				IDX[i] = C;
				k_in_expand = 0;/////////////////////////////////////////////////////////////
				for (size_t n = 0; n < count_Neighbors; n++)
				{
					super_out_find[n] = out_find[n];
				}
				while (1)
				{

					j_in_expand = super_out_find[k_in_expand];
					if (visited[j_in_expand - 1] == 0.0)
					{
						//printf("test\n");
						visited[j_in_expand - 1] = 1.0;

						memset(x_in_find, 0, CLUSTER_NUM * sizeof(x_in_find[0]));

						for (size_t m = 0; m < CLUSTER_NUM; m++)
						{
							if (D[j_in_expand - 1][m] <= boundary)
							{
								x_in_find[m] = 1.0;
							}
						}

						count_in_find_2 = 0;                           ///초기화

						memset(out_find_2, 0, CLUSTER_NUM * sizeof(out_find_2[0]));
						for (size_t m = 0; m < CLUSTER_NUM; m++)
						{
							if (x_in_find[m] != 0.0)
							{
								out_find_2[count_in_find_2] = m + 1;
								count_in_find_2 = count_in_find_2 + 1;
							}
						}
						count_Neighbors_2 = 0;
						for (size_t m = 0; m < CLUSTER_NUM; m++)
						{
							if (out_find_2[m] != 0)
							{
								count_Neighbors_2 = count_Neighbors_2 + 1;
							}
						}

						mixture = count_Neighbors + count_Neighbors_2;

						if (count_Neighbors >= minPoint)
						{
							super_k = 0;
							for (size_t q = count_Neighbors; q < (mixture); q++)
							{
								super_out_find[q] = out_find_2[super_k];
								super_k = super_k + 1;
							}
						}
						count_Neighbors = count_Neighbors + count_Neighbors_2;
					}
					if (IDX[j_in_expand - 1] == 0)
					{
						IDX[j_in_expand - 1] = C;
					}
					k_in_expand = k_in_expand + 1;
					//printf("%d\n", k_in_expand);
					if (k_in_expand + 1 > count_Neighbors)
					{
						break;
					}
				}


			}


		}
	}

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BIND_CLUSTER(void)
{
	memset(C_IDX, 0, sizeof(C_IDX));
	// 1. 노이즈 빼고 인덱스 하나씩 올리기.
	for (size_t i = 0; i < CLUSTER_NUM; i++)
	{
		if (IDX[i] == 0)
			continue;
		else
			IDX[i] = IDX[i] + 1;
	}

	// 2. 군집 개수만큼 각 배열에 해당 인덱스 담기.

	for (size_t k = 1; k <= C; k++)
	{
		int IDX_cnt = 0;
		for (size_t i = 0; i < CLUSTER_NUM; i++)
		{
			if (IDX[i] == k + 1)
			{
				C_IDX[k - 1][IDX_cnt] = i;
				IDX_cnt = IDX_cnt + 1;
			}
		}
	}

	// 3. 거리값이 1이 포함된 인덱스끼리 묶기.
	int one_cnt = 0;

	for (size_t i = 0; i < C; i++)
	{
		one_cnt = 0;
		for (size_t j = 0; j < CLUSTER_NUM; j++)
		{

			if (j > 1 && C_IDX[i][j] == 0)
				break;
			if (disCoor[C_IDX[i][j]] >= 0.99)
			{
				one_cnt = one_cnt + 1;

				if (one_cnt == 5)
				{
					for (size_t k = 0; k < CLUSTER_NUM; k++)
					{
						if (k > 0 && C_IDX[i][k] == 0)
							break;
						else
						{
							// printf("i: %d, j: %d, k: %d\n", i, j, k);
							IDX[C_IDX[i][k]] = 1;
						}
					}
					break;
				}
			}
		}
	}

	// 4. 클러스터 개수만큼 순서대로 인덱싱

	int MARKING = 2;  // 1을 바깥으로 했기 때문에 1부터 할 필요 없다.
	int marking_flag = 0;
	for (size_t i = 2; i <= C + 1; i++)
	{
		marking_flag = 0;

		for (size_t j = 0; j < CLUSTER_NUM; j++)
		{
			if (IDX[j] == i)
			{
				IDX[j] = MARKING;
				marking_flag = 1;
			}
		}

		if (marking_flag == 1)
			MARKING = MARKING + 1;
	}

	C = MARKING - 1;
}


void main_data()
{
	//printf("MAIN C: %d\n", C);
	flag_clustering = 0;
	for (size_t i = 0; i < CLUSTER_NUM; i++)
	{
		buf_xxCoor[i] = xxCoor[i];
		buf_yyCoor[i] = yyCoor[i];
	}

	if (C < 3)
	{
		delta_f = prev_deltaf;
		flag_clustering = 1;
	}
	else if (C < 4)
	{
		flag_clustering = 2;
		count_distmean = 0;
		for (int i = 0; i < C; i++)
		{
			count_cncn = 0;
			dist_Xi_mean = 0;
			for (int j = 0; j < CLUSTER_NUM; j++)
			{
				if (IDX[j] == i + 1)
				{
					Xi[count_cncn][0] = buf_xxCoor[j];
					Xi[count_cncn][1] = buf_yyCoor[j];
					dist_sample[count_cncn] = sqrt(buf_xxCoor[j] * buf_xxCoor[j] + buf_yyCoor[j] * buf_yyCoor[j]);
					dist_Xi_mean = dist_Xi_mean + dist_sample[count_cncn];
					count_cncn++;
				}
			}
			dist_Xi_mean = dist_Xi_mean / (double)count_cncn;
			if (dist_Xi_mean < 0.95)
				count_distmean++;
			buf_distmean[i][0] = dist_Xi_mean;
			buf_distmean[i][1] = i + 1.0;

		}
		for (int i = 0; i < C; i++)
		{
			count_order = 0;
			for (int d = 0; d < C; d++)
			{
				if (buf_distmean[i][0] > buf_distmean[d][0])
				{
					count_order = count_order + 1;
				}
			}
			sorted_distmean[count_order][0] = buf_distmean[i][0];
			sorted_distmean[count_order][1] = buf_distmean[i][1];

		}

		man_1 = sorted_distmean[0][1];
		man_2 = sorted_distmean[1][1];

		if (count_distmean > 0)
		{
			count_man_1 = 0.0;
			sum_man_1_x = 0.0;
			sum_man_1_y = 0.0;
			man_1_x = 0.0;
			man_1_y = 0.0;

			for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
			{
				if (IDX[cnt_go] == man_1)
				{
					sum_man_1_x = sum_man_1_x + buf_xxCoor[cnt_go];
					sum_man_1_y = sum_man_1_y + buf_yyCoor[cnt_go];
					count_man_1 = count_man_1 + 1.0;
				}
			}
			man_1_x = sum_man_1_x / (double)count_man_1;
			man_1_y = sum_man_1_y / (double)count_man_1;

			count_man_2 = 0.0;
			sum_man_2_x = 0.0;
			sum_man_2_y = 0.0;
			man_2_x = 0.0;
			man_2_y = 0.0;

			for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
			{
				if (IDX[cnt_go] == man_2)
				{
					sum_man_2_x = sum_man_2_x + buf_xxCoor[cnt_go];
					sum_man_2_y = sum_man_2_y + buf_yyCoor[cnt_go];
					count_man_2 = count_man_2 + 1.0;
				}
			}

			man_2_x = sum_man_2_x / (double)count_man_2;
			man_2_y = sum_man_2_y / (double)count_man_2;

			delta_f = atan2((man_1_y + man_2_y), (man_1_x + man_2_x)) * RAD2DEG;// -90.0; //(deg로 나옴)

			//delta_f = -delta_f;      // 오른쪽이 + 왼쪽이 - 로 변환

		}
		else
		{
			//delta_f = 0.0;
			delta_f = prev_deltaf;
		}
	}
	else //if (C < 5)
	{
		count_distmean = 0;
		for (int i = 0; i < C; i++)
		{
			count_cncn = 0;
			dist_Xi_mean = 0;
			for (int j = 0; j < CLUSTER_NUM; j++)
			{
				if (IDX[j] == i + 1)
				{
					Xi[count_cncn][0] = buf_xxCoor[j];
					Xi[count_cncn][1] = buf_yyCoor[j];
					dist_sample[count_cncn] = sqrt(buf_xxCoor[j] * buf_xxCoor[j] + buf_yyCoor[j] * buf_yyCoor[j]);
					dist_Xi_mean = dist_Xi_mean + dist_sample[count_cncn];
					count_cncn++;
				}
			}
			dist_Xi_mean = dist_Xi_mean / (double)count_cncn;

			if (dist_Xi_mean < 0.95)
				count_distmean++;
			buf_distmean[i][0] = dist_Xi_mean;
			buf_distmean[i][1] = i + 1.0;
		}
		for (int i = 0; i < C; i++)
		{
			count_order = 0;
			for (int d = 0; d < C; d++)
			{
				if (buf_distmean[i][0] > buf_distmean[d][0])
				{
					count_order = count_order + 1;
				}
			}
			sorted_distmean[count_order][0] = buf_distmean[i][0];
			sorted_distmean[count_order][1] = buf_distmean[i][1];
		}

		man_1 = sorted_distmean[0][1];
		man_2 = sorted_distmean[1][1];
		man_3 = sorted_distmean[2][1];



		if (count_distmean > 0)
		{
			count_man_1 = 0.0;
			sum_man_1_x = 0.0;
			sum_man_1_y = 0.0;
			man_1_x = 0.0;
			man_1_y = 0.0;

			for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
			{
				if (IDX[cnt_go] == man_1)
				{
					sum_man_1_x = sum_man_1_x + buf_xxCoor[cnt_go];
					sum_man_1_y = sum_man_1_y + buf_yyCoor[cnt_go];
					count_man_1 = count_man_1 + 1.0;
				}
			}
			man_1_x = sum_man_1_x / count_man_1;
			man_1_y = sum_man_1_y / count_man_1;

			count_man_2 = 0.0;
			sum_man_2_x = 0.0;
			sum_man_2_y = 0.0;
			man_2_x = 0.0;
			man_2_y = 0.0;

			for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
			{
				if (IDX[cnt_go] == man_2)
				{
					sum_man_2_x = sum_man_2_x + buf_xxCoor[cnt_go];
					sum_man_2_y = sum_man_2_y + buf_yyCoor[cnt_go];
					count_man_2 = count_man_2 + 1.0;
				}
			}

			man_2_x = sum_man_2_x / count_man_2;
			man_2_y = sum_man_2_y / count_man_2;

			count_man_3 = 0.0;
			sum_man_3_x = 0.0;
			sum_man_3_y = 0.0;
			man_3_x = 0.0;
			man_3_y = 0.0;
			double man_3_angle = 0.0;
			double man_3_angle_comp = 0.0;
			if (RecvArray_rubber[0] == LEFT_CONE) {
				for (int count_other = 2; count_other < C + 1; count_other++)
				{
					for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
					{
						if (IDX[cnt_go] == count_other)
						{
							sum_man_3_x = sum_man_3_x + buf_xxCoor[cnt_go];
							sum_man_3_y = sum_man_3_y + buf_yyCoor[cnt_go];
							count_man_3 = count_man_3 + 1.0;
						}
					}

					man_3_x = sum_man_3_x / count_man_3;
					man_3_y = sum_man_3_y / count_man_3;
					man_3_angle_comp = atan2(man_3_y, man_3_x) * RAD2DEG;
					if ((man_3_angle >= man_3_angle_comp)&&(count_other!=man_1) && (count_other != man_2))
					{
						man_3_angle = atan2(man_3_y, man_3_x) * RAD2DEG;
						man_3 = count_other;
					}

				}
			}
			else if (RecvArray_rubber[0] == RIGHT_CONE)
			{
				for (int count_other = 2; count_other < C + 1; count_other++)
				{
					for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
					{
						if (IDX[cnt_go] == count_other)
						{
							sum_man_3_x = sum_man_3_x + buf_xxCoor[cnt_go];
							sum_man_3_y = sum_man_3_y + buf_yyCoor[cnt_go];
							count_man_3 = count_man_3 + 1.0;
						}
					}

					man_3_x = sum_man_3_x / count_man_3;
					man_3_y = sum_man_3_y / count_man_3;
					man_3_angle_comp = atan2(man_3_y, man_3_x) * RAD2DEG;

					if (man_3_angle <= man_3_angle_comp && (count_other != man_1) && (count_other != man_2))
					{
						man_3_angle = atan2(man_3_y, man_3_x) * RAD2DEG;
						man_3 = count_other;
					}

				}
			}

			count_man_3 = 0.0;
			sum_man_3_x = 0.0;
			sum_man_3_y = 0.0;
			man_3_x = 0.0;
			man_3_y = 0.0;

			for (size_t cnt_go = 0; cnt_go < CLUSTER_NUM; cnt_go++)
			{
				if (IDX[cnt_go] == man_3)
				{
					sum_man_3_x = sum_man_3_x + buf_xxCoor[cnt_go];
					sum_man_3_y = sum_man_3_y + buf_yyCoor[cnt_go];
					count_man_3 = count_man_3 + 1.0;
				}
			}

			man_3_x = sum_man_3_x / count_man_3;
			man_3_y = sum_man_3_y / count_man_3;


			XX = (man_1_x * man_1_x * (man_2_y - man_3_y) + man_2_x * man_2_x * (man_3_y - man_1_y) + man_3_x * man_3_x * (man_1_y - man_2_y) - (man_1_y - man_2_y) * (man_2_y - man_3_y) * (man_3_y - man_1_y)) / (2 * (man_1_x * (man_2_y - man_3_y) + man_2_x * (man_3_y - man_1_y) + man_3_x * (man_1_y - man_2_y)));
			YY = (man_1_x * (man_2_y * man_2_y - man_3_y * man_3_y) + man_2_x * (man_3_y * man_3_y - man_1_y * man_1_y) + man_3_x * (man_1_y * man_1_y - man_2_y * man_2_y) + (man_1_x - man_2_x) * (man_2_x - man_3_x) * (man_3_x - man_1_x)) / (2 * (man_1_x * (man_2_y - man_3_y) + man_2_x * (man_3_y - man_1_y) + man_3_x * (man_1_y - man_2_y)));


			delta_f = atan2(YY, XX) * RAD2DEG;// -90.0; //(deg로 나옴)



		}
		else
		{
			delta_f = prev_deltaf;
		}
	}

	buf_C[count] = C;
	prev_deltaf = delta_f;
	buf_BFdeltaf[count] = delta_f;
	delta_f = delta_f/2.0;//////////////////////////////////// WEIGHT
	buf_AFdeltaf[count] = delta_f;
}




double(*mypdist2(double* x, double* y))[CLUSTER_NUM] {
	static double xy[CLUSTER_NUM][CLUSTER_NUM] = { 0.0, };
	static double value = 0.0;

	for (int i = 0; i < CLUSTER_NUM; i++) {
		for (int j = 0; j < CLUSTER_NUM; j++) {
			value = Distance(x, y, i, j);
			xy[i][j] = value;
		}
	}
	static double(*xy_)[CLUSTER_NUM] = xy;

	return xy_;
}
double Distance(double* x, double* y, int i, int j) { // CLEAR
	static double result = 0.0;
	//result = sqrt(pow((x[i] - x[j]), 2) + pow((y[i] - y[j]), 2));
	result = sqrt((x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]));
	return result;
}

void  lsr_MemWrite(void)
{
	register long int i, j = 0;  // register 사용하면 메모리 대신 CPU의 레지스터를 사용 -> 일반변수보다 빠르다, 반복횟수가 많을 때 유용함
	VEC_WRT(bufOFF_dist, buf_dis, simul_cnt);      // 받아들인 데이터를 버퍼에 저장
	//printf("%f\n", *OFF.dis);
	//printf("%f\n", bufOFF_dist[count][3]);
	VEC_WRT(bufOFF_angle, OFF.angle, simul_cnt);
}

void SAVE_DATA()
{
	printf("Data Writing.....\n");
	char OFFName[50];

	register long int i, j;
	FILE* offFile;

	sprintf(OFFName, "OFF_YES.csv");

	fopen_s(&offFile, OFFName, "w");

	//---------------------------------------------------------------------------//
	LOOP(i, count)
	{
		VEC_PRT_OFF(bufOFF_dist, 1.0);     /*  1 ~ 100    */
		STR_PRT_OFF("\n");
		VEC_PRT_OFF(bufOFF_angle, 1.0);   /*  1 ~ 100    */
		STR_PRT_OFF("\n");
		fprintf(offFile, "\n\n\n");

	}
	fclose(offFile);
	printf("Data Writting Finished.....\n");

}

void SaveData_Flag()
{
	dataFile = fopen("UDPdata.csv", "wt");

	if (dataFile == NULL) {
		printf("안됨..\n");
		return 1;
	}

	for (int r = 0; r < count; r++) {
		fprintf(dataFile, "%d, %d, %f, %f, %f\n",
			(int)buf_C[r], (int)buf_camFlag[r], (double)buf_BFdeltaf[r], (double)buf_AFdeltaf[r], (double)buf_speedcmd[r]);
	}

	printf("Save UDP flag Successful.\n");

	fclose(dataFile);
}


#endif // !RUBBER_C
