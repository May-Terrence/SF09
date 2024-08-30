/*
 * MySeqQueue.hpp
 *
 *  Created on: 2020年12月4日
 *      Author: 刘成吉
 */

#ifndef MODULES_ESKF_MYSEQQUEUE_HPP_
#define MODULES_ESKF_MYSEQQUEUE_HPP_

#ifdef __cplusplus
#include<iostream>
#include <malloc.h>
#include <Eigen>

using namespace Eigen;
using namespace std;
const int maxSize = 50;
template<class T>
class myQueue
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    myQueue(){};
    ~myQueue(){};
    virtual bool EnQueue(const T& x) = 0;
    virtual bool DeQueue(T& x) = 0;
    virtual bool DeQueue() = 0;
    virtual bool getFront(T& x) = 0;
    virtual bool getRear(T& x) = 0;
    virtual bool getelement(int8_t index, T& x) = 0;		//获取队列中第i个内容(相对于队头)
    virtual bool setelement(int8_t index, const T& x) = 0;		//赋值队列中第i个内容(相对于队头)
    virtual int getRearPtr()const = 0;							//获取队尾指针（索引）
    virtual int getFrontPtr()const = 0;							//获取队头指针索引
    virtual bool IsEmpty()const = 0;
    virtual bool IsFull()const = 0;
    virtual int getSize()const = 0;
    virtual int getMaxSize()const = 0;
};

template<class T>
class MySeqQueue: public myQueue<T>
{
private:
    int  front, rear;		//rear指向最后一个元素的下一个位置，front指向队头
    int maxSize;
    T *elements;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MySeqQueue(int sz=10);//构造函数
    ~MySeqQueue();//析构函数
    bool EnQueue(const T& x);//入队列
    bool DeQueue(T& x);//出队列
    bool DeQueue();//删除队头
    bool getFront(T& x);//找队头
    bool getRear(T& x);//找队尾
    bool getelement(int8_t index, T& x);//获取队列中第i个元素(相对于队头)
    bool setelement(int8_t index, const T& x);//赋值队列中第i个内容(相对于队头)
    int getRearPtr()const{return this->rear;};//获取队尾指针（索引）
    int getFrontPtr()const{return this->front;};//获取队头指针索引
    bool IsEmpty()const{return (this->rear==this->front) ? true : false;}//判空
    bool IsFull()const{return ((this->rear+1)%this->maxSize==this->front) ? true : false;}//判满
    int getSize()const{return(this->rear-this->front+this->maxSize)%this->maxSize;}//得队长
    int getMaxSize()const{return this->maxSize;}
};



template<class T>
bool MySeqQueue<T>::EnQueue(const T& x)	//入队
{
    if(!this->IsFull())
    {
        this->elements[this->rear] = x;
        this->rear = (this->rear+1)%this->maxSize;//使用求余运算符 % 实现循环增加操作
        return true;
    }
    return false;
}
template<class T>
bool MySeqQueue<T>::DeQueue(T& x)			//出队
{
    if(!this->IsEmpty())
    {
        x = this->elements[this->front];
        this->front = (this->front+1)%this->maxSize;
        return true;
    }
    return false;
}
template<class T>
bool MySeqQueue<T>::DeQueue()		//删队头
{
	if(!this->IsEmpty())
	{
		this->front = (this->front+1)%this->maxSize;
		return true;
	}
	return false;
}
template<class T>
bool MySeqQueue<T>::getFront(T& x)			//获取队头
{
    if(!this->IsEmpty())
    {
        x = this->elements[this->front];
        return true;
    }
    return false;
}
template<class T>
bool MySeqQueue<T>::getRear(T& x)				//找队尾
{
	if(!this->IsEmpty())
	{
		x = this->elements[(this->rear-1+this->maxSize)%this->maxSize];
		return true;
	}
	return false;
}
template<class T>
bool MySeqQueue<T>::getelement(int8_t index, T& x)	//获取队列中第i个元素(相对于队头)，首位为0
{
	if(!this->IsEmpty() && index >=0 && index < getSize())
	{
		x = this->elements[(this->front + index)%this->maxSize];
		return true;
	}
	return false;
}
template<class T>
bool MySeqQueue<T>::setelement(int8_t index, const T& x)		//赋值队列中第i个内容(相对于队头)，首位为0
{
	if(!this->IsEmpty() && index >=0 && index < getSize())
	{
		this->elements[(this->front + index)%this->maxSize] = x;
		return true;
	}
	return false;
}


template<class T>
MySeqQueue<T>::MySeqQueue(int sz):front(0),rear(0),maxSize(sz)			//构造函数
{
//	this->elements = (T*)malloc(sizeof(T)*maxSize);
	this->elements = new T[maxSize];
}

template<class T>
MySeqQueue<T>::~MySeqQueue()				//析构函数
{
	if(this->elements != NULL)
	{
//		free(elements);
//		elements = NULL;
		delete[] elements;
	}
}

#endif
#endif /* MODULES_ESKF_MYSEQQUEUE_HPP_ */
