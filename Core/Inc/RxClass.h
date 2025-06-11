/*
 * RxClass.h
 *
 *  Created on: Apr 18, 2025
 *      Author: Phoenix Cardwell
 *      UCONN Formula SAE 2025
 */

#ifndef INC_RXCLASS_H_
#define INC_RXCLASS_H_

#include "main.h"

class Rx_TypeDef;

class Rx_TypeDef{
public:
	Rx_TypeDef() {}
	Rx_TypeDef(uint16_t ID) : _ID(ID) { _val = 0; }
	~Rx_TypeDef() {}
	uint32_t getID() { return _ID; }
	uint32_t getValue() const { return _val; }
	void _Update(int val) { _val = val; }
private:
	int _val;
	uint32_t _ID;
};

class Rx_LapStats
{
public:
	Rx_LapStats() {}
	Rx_LapStats(uint16_t ID) :_ID(ID) { _time = 0; _lapNum = 0; }
	~Rx_LapStats() {}
	uint32_t getID() const { return _ID; }
	int getTime() const { return _time; }
	int getNum() const { return _lapNum; }
	void _Update(int time, int num) { _time = time; _lapNum = num; }
private:
	uint32_t _ID;
	int _time;
	int _lapNum;
};
// if a value needs to be calculated after can, getValue() calls the private
// Update() function to update value before returning

// perhaps the simple ones like Gear and RPM that don't need to do any further calculation don't need
// to be subclasses of Rx_TypeDef and can just be the base class
//#if 0
//class Rx_ECT : public Rx_TypeDef
//{
//public:
//	Rx_ECT() { _ID = 0x605; }
//	uint32_t getValue() override;
//	void _Update(CAN_HandleTypeDef* hcan);
//	uint32_t getID() const { return _ID; }
//private:
//	int _val;
//	uint32_t _ID;
//};
//
//class Rx_RPM : public Rx_TypeDef
//{
//public:
//	Rx_RPM() { _ID = 0x600; }
//	uint32_t getValue() override;
//	void _Update(CAN_HandleTypeDef* hcan);
//	uint32_t getID() const { return _ID; }
//private:
//	int val;
//	uint32_t _ID;
//};
//
//class Rx_oilPressure : public Rx_TypeDef
//{
//public:
//	Rx_oilPressure() { _ID = 0x60E; }
//	uint32_t getValue() override { return _val; }
//	uint32_t getID() const { return _ID; }
//	void _Update(CAN_HandleTypeDef* hcan);
//
//private:
//	int _val;
//	uint32_t _ID;
//};
//
//class Rx_vBat : public Rx_TypeDef
//{
//public:
//	uint32_t getValue() override;
//private:
//	void _Update();
//};
//
//class Rx_oilTemp : public Rx_TypeDef
//{
//public:
//	uint32_t getValue() override;
//private:
//	void _Update();
//};
//
//class Rx_lambda : public Rx_TypeDef
//{
//public:
//	uint32_t getValue() override;
//private:
//	void _Update();
//};
//#endif
#endif /* INC_RXCLASS_H_ */
