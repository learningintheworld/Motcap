#ifndef __MOVINGAVERAGEFILTER_H
#define __MOVINGAVERAGEFILTER_H

#include <stdio.h>
#include <stdlib.h>

#define WINDOW_SIZE 25

typedef struct {
    // double *data;
    double data[WINDOW_SIZE];
    int size;
    int front;
    int rear;
    double sum;
    int count;
} MovingAverageFilter;

// 全局变量，作为滤波器
extern MovingAverageFilter filter_instance[19];


void initMovingAverageFilter(MovingAverageFilter *filter, int size);

void resetMovingAverageFilter(MovingAverageFilter *filter);

void destroyMovingAverageFilter(MovingAverageFilter *filter);

void enqueue(MovingAverageFilter *filter, double value);

double dequeue(MovingAverageFilter *filter);

double getMovingAverage(MovingAverageFilter *filter);

#endif

