#include "MovingAverageFilter.h"
#include <stdio.h>

// 全局变量，作为滤波器
MovingAverageFilter filter_instance[19];


void initMovingAverageFilter(MovingAverageFilter *filter, int size) {
    // filter->data = (double*)malloc(size * sizeof(double));    	// Memory leak occurred
    for (int i = 0; i < size; ++i) {
        filter->data[i] = 0.0;
    }
    filter->size = size;
    filter->front = 0;
    filter->rear = -1;
    filter->sum = 0.0;
    filter->count = 0;
//    return filter;
}

void resetMovingAverageFilter(MovingAverageFilter *filter) {
    for (int i = 0; i < filter->size; ++i) {
        filter->data[i] = 0.0;
    }
    filter->front = 0;
    filter->rear = -1;
    filter->sum = 0.0;
    filter->count = 0;
}

void destroyMovingAverageFilter(MovingAverageFilter *filter) {
    // free(filter->data);
    // // free(filter);
    // filter->data = NULL;
}

void enqueue(MovingAverageFilter *filter, double value) {
    if (filter->rear == filter->size - 1) {
        filter->rear = -1;
    }
    filter->data[++filter->rear] = value;
    filter->sum += value;
    filter->count++;
}

double dequeue(MovingAverageFilter *filter) {
    double value = filter->data[filter->front++];
    if (filter->front == filter->size) {
        filter->front = 0;
    }
    filter->sum -= value;
    filter->count--;
    return value;
}

double getMovingAverage(MovingAverageFilter *filter) {
    // 检查队列中的元素数量是否等于窗口大小
    if (filter->count == filter->size) {
        double result =  filter->sum / filter->size;
        // 移除最旧的数据点，为下一个数据点腾出空间
        dequeue(filter);
        return result;
    } 
    else 
    {
        // 如果队列中的元素数量小于窗口大小，则只计算已有元素的平均值
        return filter->sum / filter->count;
        // return 0.0;
    }
}




