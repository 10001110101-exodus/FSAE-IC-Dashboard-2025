/*
*  Created on: Apr 18, 2025
 *      Author: Phoenix Cardwell
 *      UCONN Formula SAE 2025
 */
#ifndef __SCREENHANDLE_HPP
#define __SCREENHANDLE_HPP

#include "main.h"

#ifdef __cplusplus

#include <vector>
#include <string>
#include "RxClass.h"
#include <stdio.h>

class ScreenHandler;
class ScreenPanel;
class Rx_TypeDef;
class Rx_LapStats;

class ScreenHandler // class to handle the screen dispaly
{
public:
	ScreenHandler() { _curScreen = 0; };
	//ScreenHandler(std::vector<ScreenPanel&> screens) : _screens(screens) {}
	~ScreenHandler();

	void startScreen();
	void handle();
	void update();
	void nextScreen();
	void prevScreen();
	void addScreen(ScreenPanel* screen);

private:
	int _curScreen; // pointer to current screen

	// wait maybe i want curscreen to be an iterator
	std::vector<ScreenPanel*> _screens; // array of all screens
};


class ScreenPanel
{
public:
	virtual ~ScreenPanel() {}
	// what do I need here?
	virtual void Display() = 0; // displays to LCD screen
	virtual void titleScreen() = 0;

};

/* SPECIFIC SCREENS */

class WarmupScreen : public ScreenPanel
{
public:
	WarmupScreen() {}
	WarmupScreen(Rx_TypeDef* ECT, Rx_TypeDef* oilPressure, Rx_TypeDef* vBat,
				Rx_TypeDef* oilTemp, Rx_TypeDef* lambda) : _ECT(ECT),
														   _oilPressure(oilPressure),
														   _vBat(vBat),
														   _oilTemp(oilTemp),
														   _lambda(lambda) {}
	~WarmupScreen() {}
	void Display() override;
	void titleScreen() override;

private:
	Rx_TypeDef* _ECT;
	Rx_TypeDef* _oilPressure;
	Rx_TypeDef* _vBat;
	Rx_TypeDef* _oilTemp;
	Rx_TypeDef* _lambda;
};


class DrivingScreen : public ScreenPanel
{
public:
	DrivingScreen() {}
	DrivingScreen(Rx_TypeDef* speed, /*Rx_TypeDef* laptime,*/ Rx_TypeDef* ECT, Rx_TypeDef* limp, Rx_TypeDef* RPM) : _speed(speed),
																			 /*_laptime(laptime),*/
																			 _ECT(ECT),
																			 _limp(limp),
																			 _RPM(RPM) {}
	~DrivingScreen() {}
	void Display() override;
	void titleScreen() override;
private:
	Rx_TypeDef* _speed;
	//Rx_TypeDef*	_laptime;
	Rx_TypeDef* _ECT;
	Rx_TypeDef* _limp; // freaky
	Rx_TypeDef* _RPM;
};

class LapTimeScreen : public ScreenPanel
{
public:
	LapTimeScreen() {}
	LapTimeScreen(Rx_LapStats* laptime, Rx_TypeDef* diffBest, Rx_TypeDef* rollTime, Rx_TypeDef* bestTime, Rx_TypeDef* sessTime) : _laptime(laptime),
																 _diffBest(diffBest),
																 _rollTime(rollTime),
																 _bestTime(bestTime),
																 _sessTime(sessTime) {}
	~LapTimeScreen() {}
	void Display() override;
	void titleScreen() override;

private:
	Rx_LapStats* _laptime;
	Rx_TypeDef* _diffBest;
	Rx_TypeDef* _rollTime;
	Rx_TypeDef* _bestTime;
	Rx_TypeDef* _sessTime;
};
extern "C" {


}
#endif

#endif
